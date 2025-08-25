//
// Created by nearlab on 04/10/17.
//

#include <cv_bridge/cv_bridge.h>
#include <libdecklink/types.hpp>

#include "decklink_camera_driver.hpp"

DeckLinkCameraDriver::DeckLinkCameraDriver()
    /// member initializor - runs before constructor
    /// node handler nh and private_nh and image transport it
    /// ~ makes it private i.e. unique per node instance
    : _nh()
    , _private_nh("~")
    , _it(_nh)
{
    // Step 1 - Make sure that we have a valid DeckLink device
    const std::string decklink_device = _private_nh.param("decklink_device", std::string());
    auto result = DeckLink::Device::Get(decklink_device);
    if (!result) {
        ROS_ERROR("DeckLink device<%s> does not exist.", decklink_device.c_str());
        _nh.shutdown();
        return;
    }
    
    _device = std::move(*result);
    
    if (!_device.supports_input_format_detection()) {
        ROS_ERROR(
            "This decklink device does not support automatic input format detection. This means "
            "that it is necessary to manually set up the pixel format and display mode, something "
            "that this tool does not currently do. Please open an issue on Gitlab if you need this "
            "feature."
        );
        _nh.shutdown();
        return;
    }
    
    // Step 2 - Set up the device and make sure we're ready to go
    /// Container for Callback functions - can be called by other functions
    try {
        DeckLink::CaptureCallback cb(
            boost::bind(&DeckLinkCameraDriver::on_new_image, this, _1),
            boost::bind(&DeckLinkCameraDriver::on_video_format_changed, this, _1, _2, _3),
            boost::bind(&DeckLinkCameraDriver::on_decklink_error, this, _1)
        );
        
        _device.input()
               .enable()
               .set_callback(std::move(cb));
        
    } catch (DeckLink::decklink_driver_error& ex) {
        ROS_ERROR_STREAM("" << boost::diagnostic_information(ex));
    }
    

    // Step 3 - Set-up ROS Stuff
    
    // ROS configuration
    _camera_name = _private_nh.param("camera_name", decklink_device);
    _camera_frame = _private_nh.param("camera_frame", _camera_name);
    _camera_info_url = _private_nh.param("camera_info_url", std::string());
    
    _camera_info_mgr = std::make_unique
        <camera_info_manager::CameraInfoManager>(_nh, _camera_name, _camera_info_url);
    
    _camera_pub = _it.advertiseCamera("image_raw", 1);
    
    _start_capture = _nh.advertiseService("start_capture",
        &DeckLinkCameraDriver::on_start_capture_request, this);
    _stop_capture = _nh.advertiseService("stop_capture",
        &DeckLinkCameraDriver::on_stop_capture_request, this);
    
    // Step 4 - Start capturing images
    try {
        _device.input().start();
    } catch (const DeckLink::decklink_driver_error& ex) {
        ROS_ERROR_STREAM("" << boost::diagnostic_information(ex));
    }
}


/**
 * Convert the data from the DeckLink::Frame type to the ROS Image message with as few copies as
 * possible.
 *
 * @param frame The input video frame
 */
void DeckLinkCameraDriver::on_new_image(const DeckLink::VideoInputFrame& frame)
{
    // We assume here that the image will YUV-422, we could make this work for other image formats
    // but I don't need it personally. If you need it just open an issue on Gitlab and we'll do it
    if (frame.pixel_format() != DeckLink::PixelFormat::YUV_8Bit) {
        ROS_ERROR(
            "Unsupported pixel format <%s>. If you have a reason for not using YUV422 open an "
            "issue on Gitlab and we'll implement it.",
            to_string(frame.pixel_format()).c_str()
        );
        return;
    }
    
    /// adds the image and adjusts it
    /// needs to be updated
    const auto image_msg = boost::make_shared<sensor_msgs::Image>();
    image_msg->header.stamp    = ros::Time::now();
    image_msg->header.frame_id = _camera_frame;

    image_msg->height   = boost::numeric_cast<unsigned int>(frame.height());
    image_msg->width    = boost::numeric_cast<unsigned int>(frame.width());
    image_msg->step     = image_msg->width * 3; // RGB is 3 bytes per pixel
    image_msg->encoding = sensor_msgs::image_encodings::RGB8;
    
  
  
    // The frame contains YUV422 formatted pixels. We want to output RGB images.
    // To convert between the two we first need two OpenCV matrices
    //
    // First we create a cv::Mat wrapping the input data. This syntax should allow us to not copy
    // the data thus reducing the latency.
    cv::Mat src(image_msg->height, image_msg->width, CV_8UC2, frame.bytes());
    
    // Next we reserve enough space for the output in the vector, in this case 3 bytes per pixel
    image_msg->data.resize(image_msg->step * image_msg->height);
    
    // And create a cv::Mat wrapping that data.
    cv::Mat dst(image_msg->height, image_msg->width, CV_8UC3, image_msg->data.data());
    
    // Now we need to convert from src -> dst
    cv::cvtColor(src, dst, CV_YUV2RGB_Y422);
    
    // Finally, create the camera_info message and publish everything
    auto camera_info_msg = _camera_info_mgr->getCameraInfo();
    camera_info_msg.header = image_msg->header;
    
    /// publishes it
    _camera_pub.publish(*image_msg, camera_info_msg);
}


void DeckLinkCameraDriver::on_decklink_error(DeckLink::VideoInputError err) {
    /// checks for errors
    switch(err) {
        case DeckLink::VideoInputError::NullFrame:
        case DeckLink::VideoInputError::NoInputSource:
            ROS_ERROR(
                "Device <%s> received a bad frame, Err: %s",
                _device.get_long_name().c_str(),
                to_string(err).c_str()
            );
        
        default:
            ROS_ERROR(
                "Device <%s> received an unknown error, Err: %i",
                _device.get_long_name().c_str(), err
            );
    }
}

/// monitors video for changes, if hardware changes signal properties in field dominance, format and display mode
void DeckLinkCameraDriver::on_video_format_changed(
    DeckLink::InputFormatChangedEvent notification_event,
    const DeckLink::DisplayMode& new_display_mode,
    DeckLink::DetectedVideoInputFormatFlags detected_signal_flag
) {
    auto notification_event_raw = static_cast<BMDNotifications>(notification_event);
    
    // Check for video field changes
    if (notification_event_raw & bmdVideoInputFieldDominanceChanged) {
        const auto field_dominance = new_display_mode.get_field_dominance();
        ROS_INFO_STREAM(
            "Input field dominance changed to " << to_string(field_dominance).c_str() << " (" << DeckLink::FieldDominance_::get_description(field_dominance).c_str() << ")\n"
        );
    }
    
    // Check if the pixel format has changed
    if (notification_event_raw & bmdVideoInputColorspaceChanged) {
        ROS_INFO_STREAM(
            "Input color space changed to:"
            << DeckLink::DetectedVideoInputFormatFlags_::pretty_print(detected_signal_flag).c_str() << "\n"
        );
    }
    
    // Check if the video mode has changed
    if (notification_event_raw & bmdVideoInputDisplayModeChanged) {
        ROS_INFO_STREAM("Input display mode changed to: " << new_display_mode.get_name() << "\n");
    }
}


/// tells decklink when to start and stop capturing
void DeckLinkCameraDriver::start()
{
    _device.input().start();
}


void DeckLinkCameraDriver::stop()
{
    _device.input().stop();
}
