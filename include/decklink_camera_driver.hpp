//
// Created by nearlab on 04/10/17.
//


#pragma once

#include <opencv2/core/types.hpp>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <std_srvs/srv/empty.hpp> 

#include <libdecklink/device.hpp>
#include <libdecklink/types.hpp>

class DeckLinkCameraDriver
: public rclcpp::Node
{
    public: /* Methods */
    DeckLinkCameraDriver();

    void init_image_transport();

    /// Called ech time the decklink card receives a new image
    void on_new_image(const DeckLink::VideoInputFrame& frame);
    
    /// reports error
    void on_decklink_error(DeckLink::VideoInputError err);
    
    /// input video format changes
    void on_video_format_changed(
        DeckLink::InputFormatChangedEvent notification_event,
        const DeckLink::DisplayMode& new_display_mode,
        DeckLink::DetectedVideoInputFormatFlags detected_signal_flag
    );

    /// Service callback to start the video capture
    /// exposes /start_capture
    void on_start_capture_request(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        (void)req;
        (void)res;
        start();
    }
    
    /// Service callback to stop the video capture
    /// exposes /stop_capture
    void on_stop_capture_request(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
    {
        (void)req;
        (void)res;
        stop();
    }
    
    
private: /* Methods */
    /// Start the video capture
    void start();
    /// Stop the video capture
    void stop();


private:
    
    // Start variables
    /// The name of the camera - required to build the sensor_msgs::CameraInfo
    std::string _camera_name;
    /// The tf frame the camera should be attached to.
    std::string _camera_frame;
    /// The path to the config file containing the intrinsic configuration
    std::string _camera_info_url;
    /// Whether or not the camera is currently streaming video
    bool _stream_started = false;
    
    // ROS2 Stuff
    std::shared_ptr<image_transport::ImageTransport> _it;
    std::unique_ptr<camera_info_manager::CameraInfoManager> _camera_info_mgr;
    image_transport::CameraPublisher _camera_pub;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _start_capture;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _stop_capture;
    
    // Decklink Stuff
    DeckLink::Device _device;
    
};


