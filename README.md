# DeckLink ROS

The DeckLink ROS module exposes BlackMagic Design DeckLink video playback & capture cards to a ROS network. It leverages [libdecklink], a higher-level level interface to the BlackMagic Design SDK, to control the underlying card(s). The `libdecklink` component is already included as a submodule.

`hi_decklink_ros` is based on the previous version `decklink_ros` (https://gitlab.com/Polimi-dVRK/decklink/decklink_ros) developed at the NearLab (Politecnico di Milano) by Nima Enayati and Thibaud Chupin. This second version was developed by Thibaud Chupin and Maria-Paola Forte, and maintained by Maria-Paola Forte (Haptic Intelligence Department - Max Planck Institute for Intelligent Systems).

The `libdecklink` tools (check the status of the DeckLink card, its model, the status of the channels etc.) are not maintained anymore (and removed from this version) since this information can be obtained through the softwares provided by BlackMagic Design. If you want to access them, refer to the Polimi code (https://gitlab.com/Polimi-dVRK/decklink/libdecklink).

## Installation Instructions

This project is known to work with both ROS Kinetic and ROS Lunar, on DeckLink Duo and DeckLink Quad 2, with a clinical Intuitive da Vinci Si HD robotic system and with the dVRK (da Vinci Research Kit).

Clone the repository into your ROS workspace: 

    git clone --recursive https://github.com/MPI-IS/hi_decklink_ros.git

Build the nodes:

    catkin build hi_decklink_ros

## The `publisher` node

The publisher node reads images from one input of the DeckLink card and publishes them to ROS topic. After having the `roscore` running, open in a different terminal: 

    rosrun hi_decklink_ros publisher _decklink_device:="DeckLink [model] (input)"

This will create a `publisher` node that listens for images on from one input of your Decklink card and publishes them on the topic `/image_raw`.

To see the published image:

	rosrun image_view image_view image:="image_raw"	

The node will additionally publish `sensor_msgs::CameraInfo` messages synchronised to each image message. If the camera is uncalibrated these will be empty. If a camera calibration file is available you can pass the path to the file in the `camera_info_url` parameter. This will allow the `image_proc` family of nodes to automatically generate rectified images.

The node accepts the following parameters:

| Parameter | Description |
| --------- | ----------- |
| `decklink_device` | The name of the capture (input) interface on the DeckLink card from which to read images and the channel used. |
| `camera_name` | A name to identify the camera. The name is used to check that the right camera calibration information is being used. If the camera name in the calibration file and the name passed here differ a warning will be shown in the ROS console. If the name is not set the DeckLink device name is used instead. |
| `camera_frame` | The `tf` frame that the camera should attached to. This helps to keep point clouds generated with `stereo_image_proc` in the correct reference frame. The default value is the camera name. |
| `camera_info_url` | The location in which to locate the camera info file. This should a be an absolute file path. |

A launch file for a stereo endoscope is provided for documentation purposes in the `launch/` folder.

## The `subscriber` node

The subscriber node reads images from a ROS topic and writes them to the specified DeckLink output. This node can be used to perform keying. 

After having the `roscore` running, open in a different terminal: 

    rosrun hi_decklink_ros subscriber _decklink_device:="DeckLink [model] (input)"

This will create a `subscriber` node that monitors a ROS image topic. This node expects `BGRA8` formatted images for simplicity and will produce an error if an image with a different formatting is used. The alpha channel is required to support keying.

The node accepts the following parameters:

| Parameter | Description |
| --------- | ----------- |
| `decklink_device` | The name of the playback (output) interface on the DeckLink card onto which the images will be written and the channel used. |
| `topic` | The ROS topic from which the images will be read. |
| `image_format` | The name of the display mode to use. |
| `keying (bool)` | Whether or not to enable keying on the card. |
| `opacity (int)` | The opacity of the keyed images in the range 0 (transparent) to 255 (opaque). |

## Using keying

Keying on DeckLink devices is extremely fast (less than 1ms extra latency on average). On modern cards it is possible to re-map each individual connector so that they can be used individually for input or for output. However, to use keying you must provide an input onto which the images will be keyed; as such, you must retain a pair (the left connector is for input and the right connector is for output). The output will be the input video with the keyed image overlaid on top with the specified opacity.

The pixel format is hard coded to YUV422.

## Test
We prepared a small demo to test the subscriber node both in writing and keying mode.
After having the `roscore` running:

	rosrun hi_decklink_ros img2ros

This will create a ROS topic image (`image_ros`). You can see it with: 

	rosrun image_view image_view image:="image_ros"

Then run the subscriber node, specifying this topic:

	rosrun hi_decklink_ros subscriber _decklink_device:="DeckLink [model] (input)" _topic:="image_ros"

By default, the node will write this image.

If you want this image to be keying on the input video:
	
	rosrun hi_decklink_ros subscriber _decklink_device:="DeckLink [model] (input)" _topic:="image_ros" _keying:="True" _opacity:="150"

We designed the subscriber node, in a way that it possible to use the same node and change the two modes (write/keying) internally, using a boolean topic `function/output_write`. To understand its functioning, we suggest to check it in `subscriber.cpp`.

## Launch examples
We used these drivers to connect a workstation computer to the vision system of a clinical da Vinci Si HD surgical robot (Intuitive Inc.). In this way, we could overlay virtual content of the intraoperative images acquired by the endoscope. Examples of launch files are in the folder `launch`.
To more information about how to connect the hardware to a da Vinci robot, please refer to our paper: TO ADD FINAL TITLE AND LINK

## Citation
If you find this code useful in your research, we would kindly ask you to cite:
{
	TO ADD
}

## Acknowledgments
We thank the NearLab for developing the first version of these drivers, and in particular, Nima Enayati and Thibaud Chupin. We are also thankful to Thibaud Chupin for giving key guidance and advices in the development of this second version, to evaluate it, to improve it and to test it with a different DeckLink card and the da Vinci Research Kit (dVRK) to guarantee inter-operability.
