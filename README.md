[![Build status](https://raw.githubusercontent.com/MPI-IS-BambooAgent/sw_badges/master/badges/plans/hidecklinkros/build.svg?sanitize=true)](https://atlas.is.localnet/bamboo/browse/BAMEI-CIT/latest/)

# HI DeckLink ROS2

The **HI DeckLink ROS2** package exposes BlackMagic Design DeckLink video playback
and capture cards to a ROS2 network. It is based on the previous version for [ROS1 support](https://github.com/MPI-IS/hi_decklink_ros), which itself is based on [DeckLink ROS](https://gitlab.com/Polimi-dVRK/decklink/decklink_ros)
developed at the NearLab (Politecnico di Milano).

## Dependencies

**HI DeckLink ROS2** leverages [libdecklink](https://gitlab.com/Polimi-dVRK/decklink/libdecklink),
a higher-level level interface to the BlackMagic Design SDK,
to control the underlying card(s). This component is installed as a git submodule.

## Installation

This project has been tested with:
+ Ubuntu 24.04
+ ROS2 Jazzy Jalisco
+ DeckLink Duo and DeckLink Quad 2

Clone the repository into your ROS workspace:

    git clone git@github.com:MaxiHenk/hi_decklink_ros2.git

Start ROS2 and build your code in the hi_decklink_ros2 folder. This will ususally look like

    source /opt/ros/jazzy/setup.bash
    colcon build
    source install



## The publisher node

The publisher node reads images from one input of the DeckLink card and publishes them to ROS topic.
After having the `roscore` running, open in a different terminal:

    ros2 run hi_decklink_ros2 publisher   --ros-args   -p decklink_device:="DeckLink [model] (input)"

This will create a `publisher` node that listens for images on from one input of your DeckLink card
and publishes them on the topic `/image_raw`.

To see the published image:

	ros2 run image_view image_view --ros-args -r image:=image_raw

The node will additionally publish `sensor_msgs::CameraInfo` messages synchronised to each image message.
If the camera is uncalibrated these will be empty and you will receive an error message about it, which will not break the code.
If a camera calibration file is available you can pass the path to the file in the `camera_info_url` parameter.
This will allow the `image_proc` family of nodes to automatically generate rectified images.

The node accepts the following parameters:

| Parameter | Description |
| --------- | ----------- |
| `decklink_device` | The name of the capture (input) interface on the DeckLink card from which to read images and the channel used. |
| `camera_name` | A name to identify the camera. The name is used to check that the right camera calibration information is being used. If the camera name in the calibration file and the name passed here differ a warning will be shown in the ROS console. If the name is not set the DeckLink device name is used instead. |
| `camera_frame` | The `tf` frame that the camera should attached to. This helps to keep point clouds generated with `stereo_image_proc` in the correct reference frame. The default value is the camera name. |
| `camera_info_url` | The location in which to locate the camera info file. This should a be an absolute file path. |

A launch file for a stereo endoscope is provided for documentation purposes in the `launch/` folder. You can adapt the parameters and run it with

    ros2 launch hi_decklink_ros2 full-MPI.xml

The images will be published on the topics `/endoscope/left/image_raw` and `/endoscope/right/image_raw` respectively.

The publisher node has been tested on:
+ a dVRK (da Vinci Research Kit)

## The subscriber node

The subscriber node reads images from a ROS topic and writes them to the specified DeckLink output.
This node can be used to perform keying.

After having the `roscore` running, open in a different terminal:

    ros2 run hi_decklink_ros2 subscriber --ros-args -p decklink_device:="DeckLink [model] ([input])" -p topic:="[topic]"

This will create a `subscriber` node that monitors the ROS image topic given as input.
This node expects `BGRA8` formatted images for simplicity and will produce an error
if an image with a different formatting is used. The alpha channel is required to support keying.
To quickly test this node, please check the section "Test subscriber".

The node accepts the following parameters:

| Parameter | Description |
| --------- | ----------- |
| `decklink_device` | The name of the playback (output) interface on the DeckLink card onto which the images will be written and the channel used. |
| `topic` | The ROS topic from which the images will be read. |
| `image_format` | The name of the display mode to use. |
| `keying (bool)` | Whether or not to enable keying on the card. |
| `opacity (int)` | The opacity of the keyed images in the range 0 (transparent) to 255 (opaque). |

The subscriber node has been tested on:
+ the stereo viewer of a dVRK (da Vinci Research Kit)


## Using keying

Keying on DeckLink devices is extremely fast (less than 1ms extra latency on average).
On modern cards it is possible to re-map each individual connector
so that they can be used individually for input or for output.
However, to use keying you must provide an input onto which the images will be keyed; as such,
you must retain a pair (the left connector is for input and the right connector is for output).
The output will be the input video with the keyed image overlaid on top with the specified opacity.

The pixel format is hard coded to YUV422.

## Test subscriber

We prepared a small demo to test the subscriber node both in writing and keying mode.
We placed an image (`image.png` in the folder `sample`) that you are welcome to use. Also make sure, that the components of the DeckLink-Device actually allow overwriting the image. Otherwise, the output at the console will not appear.

After having the `roscore` running:

	ros2 run hi_decklink_ros2 img2ros --ros-args -p path:=sample/image.png

This will create a ROS topic image (`image_ros`). You can see it with:

	ros2 run image_view image_view --ros-args -r image:=image_ros

Then run the subscriber node, specifying this topic:

	ros2 run hi_decklink_ros2 subscriber --ros-args -p decklink_device:="DeckLink [model] ([input])" -p topic:="image_ros"

By default, the node will write this image.

If you want this image to be keyed on the input video:

	ros2 run hi_decklink_ros2 subscriber --ros-args -p decklink_device:="DeckLink Quad (3)" -p topic:="image_ros" -p keying:="True" -p opacity:="150"

We designed the subscriber node, in a way that it possible to use the same node and change the two modes (write/keying) internally,
using a boolean topic `function/output_write`. To understand its functioning, we suggest to check it in `subscriber.cpp`.

If the written image seems invisible, it might also be, that the alpha-values were set too small. If you run the code in Debug-Mode, you can check for this. 

Similar to the publisher, there is also a launch file, which you can run with:

    ros2 launch hi_decklink_ros2 subscribers-MPI.xml

Simply adjust the parameters there and it will simuntaneously subscribe to both cameras. 


## Launch examples

Examples of launch files are in the folder `launch`.
For more information about how to connect the hardware to a da Vinci robot,
please refer to [our paper](#citation).

## Authors

[Maria-Paola Forte](https://is.mpg.de/person/Forte),
Haptic Intelligence Department - Max Planck Institute for Intelligent Systems

[Thibaud Chupin](https://www.linkedin.com/in/thibaudchupin/)

**HI DeckLink ROS2** is based on the work done by
[Thibaud Chupin](https://www.linkedin.com/in/thibaudchupin/)
on [DeckLink ROS](https://gitlab.com/Polimi-dVRK/decklink/decklink_ros).

## Maintainers

[Maria-Paola Forte](https://is.mpg.de/person/Forte),
Haptic Intelligence Department - Max Planck Institute for Intelligent Systems

[Maximilian Henkel](https://is.mpg.de/person/mhenkel)

## License

MIT license (see LICENSE.md).

[DeckLink ROS](https://gitlab.com/Polimi-dVRK/decklink/decklink_ros) is distributed
under the MIT license (see LICENSE_declink_ros.md)

## Copyright

© 2025, Max Planck Society - Max Planck Institute for Intelligent Systems


## Notes

The `libdecklink` tools (check the status of the DeckLink card, its model, the status of the channels etc.)
are not maintained anymore since this information can be obtained
through the softwares provided by BlackMagic Design.

## Acknowledgments

We thank [Jean-Claude Passy](https://github.com/jcpassy) and the
[Software Workshop](http://is.tuebingen.mpg.de/en/software-workshop) for their help to release the code.

## Reference
If you use these drivers, please cite:
```
@article{Forte22-IJMRCAS-Design,
  title={Design of Interactive {AR} Functions for Robotic Surgery and Evaluation in Dry-Lab Lymphadenectomy},
  author={Forte, Maria-Paola and Gourishetti, Ravali and Javot, Bernard and Gomez, Ernest T. and Kuchenbecker, Katherine J.},
  journal={The International Journal of Medical Robotics and Computer Assisted Surgery},
  year={2022},
  publisher={Wiley Online Library}
}
```
