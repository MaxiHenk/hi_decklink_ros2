//
// Created by nearlab on 04/10/17.
//

#include <rclcpp/rclcpp.hpp>
#include "decklink_camera_driver.hpp"


/// main is called automatically for the function - argc is number of command-line arguments and argv argument strings
int main(int argc, char** argv) {
    
    /// initializes node in with the ROS system - contains arguments argc and argv
    /// Anonymous name thus, different instances of the node can run at the same time
    /// decklink_publisher is how the node identifies itself to the publisher
    //ros::init(argc, argv, "decklink_publisher", ros::init_options::AnonymousName);
    rclcpp::init(argc, argv);

    try {
        // Declares camera driver
        auto camera_driver = std::make_shared<DeckLinkCameraDriver>();
        camera_driver->init_image_transport();
        //Keep checking for incoming messages - loop never exits
        rclcpp::spin(camera_driver);
        
    } catch (const DeckLink::runtime_error& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("decklink_camera_driver"),
        "Ooops! An unexpected error occurred.\n%s",
        boost::diagnostic_information(ex, true).c_str()
        );
        exit(-1);
    }
    return 0;
}
