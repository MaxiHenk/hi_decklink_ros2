//
// Created by nearlab on 04/10/17.
//

#include <ros/ros.h>
#include "decklink_camera_driver.hpp"

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "decklink_publisher", ros::init_options::AnonymousName);
    
    try {
        DeckLinkCameraDriver camera_driver;
        ros::spin();
    } catch (const DeckLink::runtime_error& ex) {
        ROS_ERROR_STREAM(
            "Ooops! An unexpected error occurred. \n\n"
                << boost::diagnostic_information(ex, true)
        );
        exit(-1);
    }
    
    return 0;
}
