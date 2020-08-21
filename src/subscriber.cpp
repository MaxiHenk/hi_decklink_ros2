//
// created by tibo and paola 21/02/2018
//

/**
 * Process the received image and push it to the card.
 *
 * The function will run a series of safety check to ensure that the image is properly formatted and,
 * in debug mode, will check the transparency of the image and warn if it is too low.
 *
 * @param output The interface on which the image will be output
 * @param image The actual image data received from ROS.
 */


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "libdecklink/device.hpp"
#include "libdecklink/types.hpp"
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Bool.h>
//#include <std_msgs/Time.h> //time info

//used to adapt the keying mode to the writing mode internally (not in rosrun or in a launch file)
bool output_write, received = false; 
int keyer_opacity = 255;

using namespace DeckLink;

//ros::Time stamp = 0;

//used to adapt the keying mode to the writing mode internally (not in rosrun or in a launch file)
void writeCallback(const std_msgs::Bool &msg) {
    output_write = msg.data;
    received = true;
}

//void stampCallback(const std_msgs::Time &msg) {
//    stamp = msg.data;
//}

void OnFrameReceived(
        DeviceOutputInterface &output,
        PixelFormat pixel_format,
        const DisplayMode &display_mode,
        const sensor_msgs::Image::ConstPtr &image
) {

    //used to adapt the keying mode to the writing mode internally (not in rosrun or in a launch file)
    if (received) {
        if (output_write) {
            output.disable_keyer();
        } else {
            output.enable_keyer()
                    .set_opacity(static_cast<uint8_t>(keyer_opacity));
        }
        received = false;
    }

    auto frame = output.create_video_frame(display_mode, pixel_format);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGRA8);

    ROS_ASSERT_MSG(
            cv_ptr->image.rows == frame.height() && cv_ptr->image.cols == frame.width(),
            "Ros Image size is (%d, %d) but decklink frame is (%ld, %ld).",
            cv_ptr->image.cols, cv_ptr->image.rows, frame.width(), frame.height()
    );

    ROS_ASSERT_MSG(
            cv_ptr->image.step == frame.row_bytes(),
            "Received image and ROS image have different row sizes (%ld != %d). Are you sure that the "
                    "encoding is correct. It should provide 4 8 bit channels.",
            cv_ptr->image.step, frame.row_bytes()
    );

#ifdef DEBUG
    double accumulated_transparency = 0.0;
    for (unsigned int i = 3; i < image->data.size(); i += 4) {
        accumulated_transparency += image->data[i];
    }

    auto mean_transparency = accumulated_transparency / std::floor(image->data.size() / 4);
    ROS_ASSERT_MSG(
        mean_transparency > 10.0,
        "The mean alpha channel value on the frame is low (~ %d), because of this the image may "
        "appear invisible on the output.", mean_transparency
    );
#endif

    frame.load(cv_ptr->image.data, static_cast<size_t>(cv_ptr->image.rows * cv_ptr->image.step));

    output.display_video_frame(frame);

    //double execution_time = (ros::Time::now() - stamp).toNSec() * 1e-6;
    //ROS_INFO_STREAM("execution time:   " << execution_time); // "Execution time platform (ms): "
}

int main(int argc, char **argv) {

    try {

        ros::init(argc, argv, "decklink_subscriber", ros::init_options::AnonymousName);

        ros::NodeHandle node;
        ros::NodeHandle private_node("~");

        image_transport::ImageTransport transport(node);
        ros::Rate loop_rate(60);

        // Device
        std::string device_name;
        private_node.getParam("decklink_device", device_name);

        if (device_name.empty()) {
            ROS_ERROR_STREAM(
                    "No device specified. You must specify the device from which the images will be "
                            "captured. The device name can be retrieved by running the `list_devices` tool"
            );
            return -1;
        }

        // Topic
        std::string topic;
        private_node.getParam("topic", topic);
        if (topic.empty()) {
            ROS_ERROR_STREAM(
                    "" << "No input topic set. You must set the topic that will be providing images. This "
                       << "can be done by setting the '_topic:=?' parameter in the command or in the "
                       << "launch file."
            );
        }

        std::string format;
        private_node.getParam("image_format", format);
        if (format.empty()) {
            format = "HD1080i5994";
            ROS_WARN_STREAM(
                    "" << "No format set. The video format used when reading the images should be set with "
                       << "the '_display_format:=?' argument in the command or the launch file. The format will be "
                       << "set to 'HD1080i5994' for the lifetime of this node."
            );
        }

        ROS_INFO_STREAM(
                "" << "Starting <" << ros::this_node::getName().c_str() << "> publishing images received "
                   << "from <" << topic.c_str() << "> to device <" << device_name.c_str() << "> formatted "
                   << "as <" << format.c_str() << ">"
        );

        auto has_device = Device::Get(device_name);
        if (!has_device) {
            ROS_ERROR_STREAM(
                    "" << "Error: Unable to locate device named: " << device_name << ".\n"
            );
            return -1;
        }

        auto &device = *has_device;

        const auto image_format = DeckLink::to_ImageFormat(format);
        const auto pixel_format = DeckLink::PixelFormat::BGRA_8Bit;

        const auto res = device.output().get_display_mode(image_format);
        if (!res) {
            ROS_ERROR_STREAM(
                    "" << "Image format \"" << to_string(image_format) << "\" is not "
                       << "supported by this device"
            );
            return -1;
        }

        const auto display_mode = *res;
        device.output()
                .enable(display_mode, pixel_format, VideoOutputFlags::Default);

        bool use_keying = false;
        private_node.getParam("keying", use_keying);

        if (use_keying) {
            private_node.getParam("opacity", keyer_opacity);
            if (keyer_opacity < 0 || keyer_opacity > 255) {
                ROS_ERROR_STREAM(
                        "" << "Keyer opacity of " << keyer_opacity << " is out of range. Valid values are in "
                           << "the inclusive range [0; 255]"
                );
                return -1;
            }
            device.output().enable_keyer()
                    .set_opacity(static_cast<uint8_t>(keyer_opacity));
        }

        ROS_INFO("Starting ROS Subscriber");
        auto frame_cb = boost::bind(OnFrameReceived,
                                    std::ref(device.output()), pixel_format, std::ref(display_mode), _1);
        image_transport::Subscriber subscriber = transport.subscribe(topic, 1, frame_cb);

        //used to adapt the keying mode to the writing mode internally (not in rosrun or in a launch file)
        ros::Subscriber sub_full = node.subscribe("function/output_write", 1, writeCallback);
        
	//ros::Subscriber sub_stamp = node.subscribe("time_stamp", 1, stampCallback);

        ROS_INFO("Subscriber ready. Waiting for frames ...");

        ros::spin();
        loop_rate.sleep();
    } catch (const DeckLink::decklink_driver_error &ex) {
        std::cout << "\nA low-level DeckLink Driver command failed: \n";

        if (const auto device = boost::get_error_info<decklink_device>(ex)) {
            std::cout << "  DeckLink::Device     : " << *device << "\n";
        }

        if (const auto command = boost::get_error_info<decklink_command>(ex)) {
            std::cout << "  Command Name         : " << *command << "\n";
        }

        if (const auto return_code = boost::get_error_info<decklink_error_code>(ex)) {
            std::cout << "  Command Return Status: " << to_string(*return_code) << "\n";
        }

        if (const auto message = boost::get_error_info<errmsg>(ex)) {
            std::cout << "  Command Error Message: " << *message << "\n";
        }

        std::cout << "\n === Complete Error Information === \n"
                  << boost::diagnostic_information(ex, true) << "\n";

    } catch (const DeckLink::runtime_error &ex) {
        std::cout << "\n"
                  << "An error occurred! \n"
                  << boost::diagnostic_information(ex) << "\n";
    }

    return 0;
}

