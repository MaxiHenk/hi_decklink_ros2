//
// Created by paola on 21/04/2020.
//

// Create a ROS topic image to test the subscriber node

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.hpp>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    // this doesnt work
    //ros::NodeHandle n, pn("~"); // get references to the node
    // private Node Handles are handled seperatley now. You can just access them with this

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("img2ros");

	image_transport::ImageTransport it(node);

    //Load the image that will be written/overlaid
    node->declare_parameter<std::string>("path", "");
	std::string path;
    path = node->get_parameter("path").as_string();


    if (path.empty()) {
        RCLCPP_ERROR(node->get_logger(),
            "No image path specified. Use --ros-args -p path:=/path/to/image.png");
        return -1;
	}

	cv::Mat image = cv::imread(path, cv::IMREAD_UNCHANGED);
    if (image.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load image: %s", path.c_str());
        return -1;
    }
	cv::Mat image_ros;
	cv::cvtColor(image, image_ros, CV_BGR2BGRA, 4);
    
	// Publisher
    // pub_img = it.advertise("image_ros", 1);
    auto pub_img = node->create_publisher<sensor_msgs::msg::Image>("image_ros", 1);

    // set frame rate for ros
    rclcpp::Rate loop_rate(60);
    
    // main function
    while (node && rclcpp::ok()) {
        auto img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgra8", image_ros).toImageMsg();
        pub_img->publish(*img);
        rclcpp::spin_all(node, 0s);
        loop_rate.sleep();
    }
    return 0;
}



