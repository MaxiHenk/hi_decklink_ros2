//
// Originally created by nearlab on 04/10/17 - updated to ROS2
//

#include <rclcpp/rclcpp.hpp>
#include "decklink_camera_driver.hpp"

// Wrapper function for the publisher
int main(int argc, char** argv) {

    rclcpp::init(argc, argv);

    try {
        auto camera_driver = std::make_shared<DeckLinkCameraDriver>();
        camera_driver->init_image_transport();
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
