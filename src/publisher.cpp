//
// Created by nearlab on 04/10/17.
//

#include <ros/ros.h>
#include "decklink_camera_driver.hpp"


/// main is called automatically for the function - argc is number of command-line arguments and argv argument strings
int main(int argc, char** argv) {
    
    /// initializes node in with the ROS system - contains arguments argc and argv
    /// Anonymous name thus, different instances of the node can run at the same time
    /// decklink_publisher is how the node identifies itself to the publisher
    ros::init(argc, argv, "decklink_publisher", ros::init_options::AnonymousName);
    
    try {
        // Declares camera driver
        DeckLinkCameraDriver camera_driver;
        //Keep checking for incoming messages - loop never exits
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
