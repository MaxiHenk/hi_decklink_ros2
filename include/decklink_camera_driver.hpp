//
// Created by nearlab on 04/10/17.
//


#pragma once

#include <opencv2/core/types.hpp>

#include <ros/node_handle.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <std_srvs/Empty.h>

#include <libdecklink/device.hpp>
#include <libdecklink/types.hpp>

class DeckLinkCameraDriver
{
public: /* Methods */
    
    DeckLinkCameraDriver();
    
public: /* Callbacks */
    
    /// Called ech time the decklink card receives a new image
    void on_new_image(const DeckLink::VideoInputFrame& frame);
    
    void on_decklink_error(DeckLink::VideoInputError err);
    
    void on_video_format_changed(
        DeckLink::InputFormatChangedEvent notification_event,
        const DeckLink::DisplayMode& new_display_mode,
        DeckLink::DetectedVideoInputFormatFlags detected_signal_flag
    );
    
    /// Service callback to start the video capture
    bool on_start_capture_request(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        start();
        return true;
    };
    
    /// Service callback to stop the video capture
    bool on_stop_capture_request(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        stop();
        return true;
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
    
    // ROS Stuff
    ros::NodeHandle _nh;
    ros::NodeHandle _private_nh;
    image_transport::ImageTransport _it;
    
    image_transport::CameraPublisher _camera_pub;
    std::unique_ptr<camera_info_manager::CameraInfoManager> _camera_info_mgr;
    
    ros::ServiceServer _start_capture;
    ros::ServiceServer _stop_capture;
    
    // Decklink Stuff
    DeckLink::Device _device;
    
};


