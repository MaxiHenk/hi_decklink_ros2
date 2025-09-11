/**
 * Process the received image and push it to the card.
 *
 * The function will run a series of safety check to ensure that the image is properly formatted and,
 * in debug mode, will check the transparency of the image and warn if it is too low.
 *
 * @param output The interface on which the image will be output
 * @param image The actual image data received from ROS.
 */


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include "libdecklink/device.hpp"
#include "libdecklink/types.hpp"
#include <cv_bridge/cv_bridge.hpp>

#include <std_msgs/msg/bool.hpp>
//#include <std_msgs/Time.h> //time info

using std_msgs::msg::Bool;
using sensor_msgs::msg::Image;

//used to adapt the keying mode to the writing mode internally (not in rosrun or in a launch file)
bool output_write, received = false;
int keyer_opacity = 255;

using namespace DeckLink;

//ros::Time stamp = 0;

//used to adapt the keying mode to the writing mode internally (not in rosrun or in a launch file)
void writeCallback(const Bool::SharedPtr msg) {
    output_write = msg->data;
    received = true;
}

//void stampCallback(const std_msgs::Time &msg) {
//    stamp = msg.data;
//}

void OnFrameReceived(
        DeviceOutputInterface &output,
        PixelFormat pixel_format,
        const DisplayMode &display_mode,
        const sensor_msgs::msg::Image::SharedPtr &image
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

    /// creates empty video_frame
    auto frame = output.create_video_frame(display_mode, pixel_format);
    //converts ROS to opencv
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGRA8);

    if (cv_ptr->image.rows != frame.height() || cv_ptr->image.cols != frame.width()) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("subscriber"),
            "Ros Image size is (" << cv_ptr->image.rows << ", " << cv_ptr->image.cols
            << ") but decklink frame is (" << frame.height() << ", " << frame.width() << ")"
        );
    }

    if (cv_ptr->image.step != static_cast<size_t>(frame.row_bytes())){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("subscriber"),
            "Received image and ROS image have different row sizes (" << cv_ptr->image.step << 
            "!=" << frame.row_bytes()<<"). Are you sure that the encoding is correct. It should provide"
            "4 8 bit channels."
        );
    }

/// debug check for transparency
#ifdef DEBUG
    double accumulated_transparency = 0.0;
    for (unsigned int i = 3; i < image->data.size(); i += 4) {
        accumulated_transparency += image->data[i];
    }
    auto mean_transparency = accumulated_transparency / std::floor(image->data.size() / 4);
    if(mean_transparency <= 10.0)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("subscriber"),
            "The mean alpha channel value on the frame is low (~"<< mean_transparency 
            << "), because of this the image may appear invisible on the output."
        );
    }
#endif

    frame.load(cv_ptr->image.data, static_cast<size_t>(cv_ptr->image.rows * cv_ptr->image.step));
    //sends frame to the Deck Link Card, thus it appears in the video output device
    output.display_video_frame(frame);

    //double execution_time = (ros::Time::now() - stamp).toNSec() * 1e-6;
    //ROS_INFO_STREAM("execution time:   " << execution_time); // "Execution time platform (ms): "
}

int main(int argc, char **argv) {

    try {

        //ros::init(argc, argv, "decklink_subscriber", ros::init_options::AnonymousName);

        /// Adds node to advertise topics etc - main interface to ROS
        //ros::NodeHandle node;
        //ros::NodeHandle private_node("~");
        rclcpp::init(argc, argv);
        auto node = rclcpp::Node::make_shared("decklink_subscriber");

        /// transport, that can publish and subscribt to images
        //image_transport::ImageTransport transport(node);
        /// loop frequency
        rclcpp::Rate loop_rate(60);

        // Device
        //std::string device_name;
        //private_node.getParam("decklink_device", device_name);

        node->declare_parameter<std::string>("decklink_device", "");
	    std::string device_name;
        device_name = node->get_parameter("decklink_device").as_string();

        if (device_name.empty()) {
            RCLCPP_ERROR(node->get_logger(),
                "No device specified. You must specify the device from which the images will be "
                "captured. The device name can be retrieved by running the `list_devices` tool"
            );
            return -1;
        }

        // Topic
        //std::string topic;
        //private_node.getParam("topic", topic);
        node->declare_parameter<std::string>("topic", "");
	    std::string topic;
        topic = node->get_parameter("topic").as_string();
        if (topic.empty()) {
            RCLCPP_ERROR(node->get_logger(),
                "No input topic set. You must set the topic that will be providing images. This "
                "can be done by setting the '_topic:=?' parameter in the command or in the "
                "launch file."
            );
            return -1;
        }

        node->declare_parameter<std::string>("image_format", "");
	    std::string format;
        format = node->get_parameter("image_format").as_string();
        //std::string format;
        //private_node.getParam("image_format", format);
        if (format.empty()) {
            format = "HD1080i5994";
            RCLCPP_WARN(node->get_logger(),
                "No format set. The video format used when reading the images should be set with "
                "the '_display_format:=?' argument in the command or the launch file. The format will be "
                "set to 'HD1080i5994' for the lifetime of this node."
            );
        }

        RCLCPP_INFO_STREAM(node->get_logger(),
            "Starting <" << node->get_name() << "> publishing images received "
            << "from <" << topic << "> to device <" << device_name << "> formatted "
            << "as <" << format << ">"
        );

        auto has_device = Device::Get(device_name);
        if (!has_device) {
            RCLCPP_ERROR_STREAM(node->get_logger(),
                "Error: Unable to locate device named: " << device_name << ".\n"
            );
            return -1;
        }

        auto &device = *has_device;

        const auto image_format = DeckLink::to_ImageFormat(format);
        const auto pixel_format = DeckLink::PixelFormat::BGRA_8Bit;

        const auto res = device.output().get_display_mode(image_format);
        if (!res) {
            RCLCPP_ERROR_STREAM(node->get_logger(),
                "Image format \"" << to_string(image_format) << "\" is not "
                "supported by this device"
            );
            return -1;
        }

        const auto display_mode = *res;
        device.output().enable(display_mode, pixel_format, VideoOutputFlags::Default);

        
        //private_node.getParam("keying", use_keying);
        node->declare_parameter<bool>("keying", false);
	    bool use_keying;
        use_keying = node->get_parameter("keying").as_bool();

        if (use_keying) {
            node->declare_parameter<int>("opacity", keyer_opacity);
            keyer_opacity = node->get_parameter("opacity").as_int();
            if (keyer_opacity < 0 || keyer_opacity > 255) {
                RCLCPP_ERROR_STREAM(node->get_logger(),
                    "Keyer opacity of " << keyer_opacity << " is out of range. Valid values are in "
                    "the inclusive range [0; 255]"
                );
                return -1;
            }
            device.output().enable_keyer()
                    .set_opacity(static_cast<uint8_t>(keyer_opacity));
        }

        RCLCPP_INFO_STREAM(node->get_logger(),"Starting ROS Subscriber");
        //auto frame_cb = std::bind(OnFrameReceived,
        //                            std::ref(device.output()), pixel_format, std::ref(display_mode), std::placeholders::_1);
        //image_transport::Subscriber subscriber = transport.subscribe(topic, 1, frame_cb);
        //auto subscriber = node->create_subscription<sensor_msgs::msg::Image>("image_ros", rclcpp::QoS(1), frame_cb);
        
        auto subscriber = node->create_subscription<sensor_msgs::msg::Image>(
            topic,
            rclcpp::QoS(1),
            [&device, pixel_format, &display_mode](const sensor_msgs::msg::Image::SharedPtr image) {
                OnFrameReceived(device.output(), pixel_format, display_mode, image);
            }
        );
        //used to adapt the keying mode to the writing mode internally (not in rosrun or in a launch file)
        //ros::Subscriber sub_full = node.subscribe("function/output_write", 1, writeCallback);
        auto sub_full = node->create_subscription<std_msgs::msg::Bool>("function/output_write", rclcpp::QoS(1), writeCallback);
	    //ros::Subscriber sub_stamp = node.subscribe("time_stamp", 1, stampCallback);

        RCLCPP_INFO_STREAM(node->get_logger(),"Subscriber ready. Waiting for frames ...");

        rclcpp::spin(node);
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

