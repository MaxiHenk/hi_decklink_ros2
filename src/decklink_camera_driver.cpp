//
// Originally created by nearlab on 04/10/17 - updated to ROS2
//

#include <cv_bridge/cv_bridge.hpp>
#include <libdecklink/types.hpp>
#include <boost/numeric/conversion/cast.hpp>

#include "decklink_camera_driver.hpp"

using std_srvs::srv::Empty;

DeckLinkCameraDriver::DeckLinkCameraDriver()
: rclcpp::Node("decklink_camera_driver")
{   

    RCLCPP_INFO_STREAM(this->get_logger(),
    "Starting to initialize parameters");
    ///initialize
    this->declare_parameter<std::string>("camera_name", "decklink_camera");
    this->declare_parameter<std::string>("camera_frame", "camera_frame");
    this->declare_parameter<std::string>("camera_info_url", "");

    this->get_parameter("camera_name", _camera_name);
    this->get_parameter("camera_frame", _camera_frame);
    this->get_parameter("camera_info_url", _camera_info_url);

    RCLCPP_INFO_STREAM(this->get_logger(),
    "Finished initializing parameters");

    // Step 1 - Make sure that we have a valid DeckLink device
    this->declare_parameter<std::string>("decklink_device", "");
	std::string decklink_device;
    decklink_device = this->get_parameter("decklink_device").as_string();


    RCLCPP_INFO_STREAM(this->get_logger(),
    "Starting to initialize DeckLink");
    auto result = DeckLink::Device::Get(decklink_device);
    if (!result) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
        "DeckLink device<" << decklink_device.c_str()<<"> does not exist.");
        return;
    }
    
    _device = std::move(*result);
    
    if (!_device.supports_input_format_detection()) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
            "This decklink device does not support automatic input format detection. This means "
            "that it is necessary to manually set up the pixel format and display mode, something "
            "that this tool does not currently do. Please open an issue on Gitlab if you need this "
            "feature."
        );
        return;
    }

    RCLCPP_INFO_STREAM(this->get_logger(),
    "Starting to initialize Callbacks");
    
    // Step 2 - Set up the device and make sure we're ready to go
    /// Container for Callback functions - can be called by other functions
    try {
        DeckLink::CaptureCallback cb(
            std::bind(&DeckLinkCameraDriver::on_new_image, this, std::placeholders::_1),
            std::bind(&DeckLinkCameraDriver::on_video_format_changed, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
            std::bind(&DeckLinkCameraDriver::on_decklink_error, this, std::placeholders::_1)
        );
        
        _device.input()
               .enable()
               .set_callback(std::move(cb));
        
    } catch (DeckLink::decklink_driver_error& ex) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
            boost::diagnostic_information(ex));
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
    "Starting to initialize ROS-Stuff");
    

    // Step 3 - Set-up ROS Stuff
    _camera_info_mgr = std::make_unique
        <camera_info_manager::CameraInfoManager>(this, _camera_name, _camera_info_url);
    
    
    _start_capture = this->create_service<std_srvs::srv::Empty>(
        "start_capture",
        std::bind(&DeckLinkCameraDriver::on_start_capture_request,
                this, std::placeholders::_1, std::placeholders::_2)
    );

    _stop_capture = this->create_service<std_srvs::srv::Empty>(
        "stop_capture",
        std::bind(&DeckLinkCameraDriver::on_stop_capture_request,
                this, std::placeholders::_1, std::placeholders::_2)
    );
    RCLCPP_INFO_STREAM(this->get_logger(),
    "Set up capturing images");
    // Step 4 - Start capturing images
    try {
        _device.input().start();
    } catch (const DeckLink::decklink_driver_error& ex) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
        boost::diagnostic_information(ex));
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
    "Finished capturing images");
}



void DeckLinkCameraDriver::init_image_transport() {
        RCLCPP_INFO_STREAM(this->get_logger(),
        "Initialized image transport and publisher");
        _it = std::make_unique<image_transport::ImageTransport>(shared_from_this());
        _camera_pub = _it->advertiseCamera("image_raw", 1);
    }


void DeckLinkCameraDriver::on_new_image(const DeckLink::VideoInputFrame& frame)
{
    // We assume here that the image will have the format YUV-422, we could make this work for other image formats
    // but I don't need it personally. If you need it just open an issue on Gitlab and we'll do it
    if (frame.pixel_format() != DeckLink::PixelFormat::YUV_8Bit) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
            "Unsupported pixel format <" << to_string(frame.pixel_format())
            << ">. If you have a reason for not using YUV422, open an issue on Gitlab."
        );
        return;
    }
    
    /// adds the image and adjusts it
    const auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
    rclcpp::Time t = this->now();
    image_msg->header.stamp    = t;
    image_msg->header.frame_id = _camera_frame;

    image_msg->height   = boost::numeric_cast<unsigned int>(frame.height());
    image_msg->width    = boost::numeric_cast<unsigned int>(frame.width());
    image_msg->step     = image_msg->width * 3; // RGB is 3 bytes per pixel
    image_msg->encoding = sensor_msgs::image_encodings::RGB8;
    
    // The frame contains YUV422 formatted pixels. We want to output RGB images.
    // To convert between the two we first need two OpenCV matrices
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
    _camera_pub.publish(*image_msg, camera_info_msg);
}


void DeckLinkCameraDriver::on_decklink_error(DeckLink::VideoInputError err) {
    /// checks for errors
    switch(err) {
        case DeckLink::VideoInputError::NullFrame:
        case DeckLink::VideoInputError::NoInputSource:
            RCLCPP_ERROR_STREAM(this->get_logger(),
                "Device <"<<_device.get_long_name().c_str()<<"> received a bad frame, Err:" << to_string(err).c_str()                
            );
        
        default:
            RCLCPP_ERROR_STREAM(this->get_logger(),
                "Device <"<<_device.get_long_name().c_str()<<"> received an unknown error, Err:" << to_string(err).c_str()                
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
        RCLCPP_INFO_STREAM(this->get_logger(),
            "Input field dominance changed to " << to_string(field_dominance).c_str() << " (" << DeckLink::FieldDominance_::get_description(field_dominance).c_str() << ")\n"
        );
    }
    
    // Check if the pixel format has changed
    if (notification_event_raw & bmdVideoInputColorspaceChanged) {
        RCLCPP_INFO_STREAM(this->get_logger(),
            "Input color space changed to:"
            << DeckLink::DetectedVideoInputFormatFlags_::pretty_print(detected_signal_flag).c_str() << "\n"
        );
    }
    
    // Check if the video mode has changed
    if (notification_event_raw & bmdVideoInputDisplayModeChanged) {
        RCLCPP_INFO_STREAM(this->get_logger(),
        "Input display mode changed to: " << new_display_mode.get_name() << "\n");
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
