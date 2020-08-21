//
// Created by paola on 21/04/2020.
//

// Create a ROS topic image to test the subscriber node

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "img2ros");
    ros::NodeHandle n, pn("~"); // get references to the node

	image_transport::ImageTransport it(n);
	image_transport::Publisher pub_img;

    //Load the image that will be written/overlaid
	std::string path;
    pn.getParam("path", path);
    if (path.empty()) {
		ROS_ERROR_STREAM("No image path specified. You must specify the path of the image that you want to write/overlay with _path:=/path/to/your/image.png");
		return -1;
	}

	cv::Mat image = cv::imread(path, cv::IMREAD_UNCHANGED);
	cv::Mat image_ros;
	cv::cvtColor(image, image_ros, CV_BGR2BGRA, 4);
    
	// Publisher
    pub_img = it.advertise("image_ros", 1);

    // set frame rate for ros
    ros::Rate loop_rate(60);

    // main function
    while (n.ok()) {
		sensor_msgs::ImagePtr img = cv_bridge::CvImage(std_msgs::Header(), "bgra8",image_ros).toImageMsg();
        pub_img.publish(img);
        ros::spinOnce();
        loop_rate.sleep();
    }
}



