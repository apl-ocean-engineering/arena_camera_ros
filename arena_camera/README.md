# ROS-Driver for Lucid Cameras using the Arena SDK

**Project Status:**  This software is a work in progress.  At present the streaming node is functional with the sample cameras which are available to us (a [PHX122S](https://thinklucid.com/product/phoenix-12-2-mp-imx226/) and a [TRI054S](https://thinklucid.com/product/triton-5-mp-imx490/) for testing HDR modes).   The polled node is not functional.

**developed by Magazino GmbH, using the arena Software Camera Suite by Lucid AG**

Based on the upstream [Arena Camera Driver](https://github.com/lucidvisionlabs/arena_camera_ros) by Magazino GmbH, this node has been heavily modified for structure and stability.

This package offers many functions of the Lucid arena API inside the ROS-Framwork.   This version focuses on the GigE cameras although it should be largely applicable to USB 3.0 cameras (but we don't have the hardware to test).


This package includes two node/nodelets:

* `ArenaCameraStreamingNodelet`: Publishes images to `/image_raw/` as they are received from the camera.  The camera can be triggered using an external hardware trigger, its own internal timer, a software timer in the node, or manually triggered through a service.   However, in the latter case, there is no direct link between the service trigger and the resulting image (i.e., you send the trigger, and later an image is published to `/image_raw`).  The matching node is `streaming_arena_camera`

* `ArenaCameraPolledNodelet`: Exposes an actionlib interfaces which allows the *synchronous* capture of images, where each trigger of the camera returns a single image, and the capture properties (exposure, brightness, etc) can be explicitly set for each image.   The matching node is `polled_arena_camera`

In both cases, camera calibration information can optionally be published through the ROS-standard `/camera_info` mechanism.

Camera settings including binning (in x and y direction), exposure, gain, gamma and brightness can be done using provided 'set_*' services.

The package opens either a predefined camera (using either the given `device_user_id` or `serial_number` parameters) or, if no camera id is predefined the first camera device it can find.

# Installation

The package has been tested for ROS-Noetic.

The Arena SDK must be installed.  See the instructions for [Lucid Vision](https://support.thinklucid.com/using-ros-for-linux/) for general instructions on installing Arena.

******
**Parameters**
******

All parameters are listed in the default config file:  ``config/default.yaml``

**Common parameters**

- **camera_frame**
  The tf frame under which the images were published

- **device_user_id**
  The DeviceUserID of the camera. If empty, the first camera found in the device list will be used

- **camera_info_url**
  The CameraInfo URL (Uniform Resource Locator) where the optional intrinsic camera calibration parameters are stored. This URL string will be parsed from the CameraInfoManager:
  http://docs.ros.org/api/camera_info_manager/html/classcamera__info__manager_1_1CameraInfoManager.html#details

- **image_encoding**
  The encoding of the pixels -- channel meaning, ordering, size taken from the list of strings in include/sensor_msgs/image_encodings.h. The supported encodings are 'mono8', 'bgr8', 'rgb8', 'bayer_bggr8', 'bayer_gbrg8' and 'bayer_rggb8'.
  Default values are 'mono8' and 'rgb8'

- **binning_x & binning_y**
  Binning factor to get downsampled images. It refers here to any camera setting which combines rectangular neighborhoods of pixels into larger "super-pixels." It reduces the resolution of the output image to (width / binning_x) x (height / binning_y). The default values binning_x = binning_y = 0 are considered the same as binning_x = binning_y = 1 (no subsampling).

- **downsampling_factor_exposure_search**
  To speed up the exposure search, the mean brightness is not calculated on the entire image, but on a subset instead. The image is downsampled until a desired window hight is reached. The window hight is calculated out of the image height divided by the downsampling_factor_exposure search

- **frame_rate**
  The desired publisher frame rate if listening to the topics. This parameter can only be set once at start-up. Calling the GrabImages-Action can result in a higher frame rate.

- **shutter_mode**
  Set mode of camera's shutter if the value is not empty. The supported modes are 'rolling', 'global' and 'global_reset'.
  Default value is '' (empty)

**Image Intensity Settings**

The following settings do **NOT** have to be set. Each camera has default values which provide an automatic image adjustment resulting in valid images

- **exposure**
  The exposure time in microseconds to be set after opening the camera.

- **gain**
  The target gain in percent of the maximal value the camera supports. For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so called 'device specific units'.

- **gamma**
  Gamma correction of pixel intensity. Adjusts the brightness of the pixel values output by the camera's sensor to account for a non-linearity in the human perception of brightness or of the display system (such as CRT).

- **brightness**
  The average intensity value of the images. It depends the exposure time as well as the gain setting. If '**exposure**' is provided, the interface will try to reach the desired brightness by only varying the gain. (What may often fail, because the range of possible exposure values is many times higher than the gain range). If '**gain**' is provided, the interface will try to reach the desired brightness by only varying the exposure time. If '**gain**' AND '**exposure**' are given, it is not possible to reach the brightness, because both are assumed to be fix.

- **brightness_continuous**
  Only relevant, if '**brightness**' is set: The brightness_continuous flag controls the auto brightness function. If it is set to false, the brightness will only be reached once. Hence changing light conditions lead to changing brightness values. If it is set to true, the given brightness will be reached continuously, trying to adapt to changing light conditions. This is only possible for values in the possible auto range of the arena API which is e.g. [50 - 205] for acA2500-14um and acA1920-40gm

- **exposure_auto & gain_auto**
  Only relevant, if '**brightness**' is set: If the camera should try to reach and / or keep the brightness, hence adapting to changing light conditions, at least one of the following flags must be set. If both are set, the interface will use the profile that tries to keep the gain at minimum to reduce white noise. The exposure_auto flag indicates, that the desired brightness will be reached by adapting the exposure time. The gain_auto flag indicates, that the desired brightness will be reached by adapting the gain.

**Optional and device specific parameter**

- **gige/mtu_size**
  The MTU size. Only used for GigE cameras. To prevent lost frames configure the camera has to be configured with the MTU size the network card supports. A value greater 3000 should be good (1500 for RaspberryPI)

- **gige/inter_pkg_delay**
  The inter-package delay in ticks. Only used for GigE cameras. To prevent lost frames it should be greater 0. For most of GigE-Cameras, a value of 1000 is reasonable. For GigE-Cameras used on a RaspberryPI this value should be set to 11772.


# Questions

Please provide your questions via http://answers.ros.org/questions/ and tag them with **arena_camera**


# LICENSE

We retain the BSD 3-Clause license supplied with the upstream [arena_camera_ros](https://github.com/lucidvisionlabs/arena_camera_ros/blob/master/catkin_ws/src/arena_camera/LICENSE).   See the [LICENSE](LICENSE) file.
