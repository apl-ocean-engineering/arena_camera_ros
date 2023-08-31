## Arena Camera Driver for ROS1

This repo contains a ROS1 driver for Lucidvision machine vision camera using their ["Arena" SDK for Ubuntu](https://thinklucid.com/downloads-hub/).

This particular version is forked from the upstream [Arena Camera Driver](https://github.com/lucidvisionlabs/arena_camera_ros) from LucidVision and contains significant refactoring for functionality / readability.

See [arena_camera_ros2](https://github.com/apl-ocean-engineering/arena_camera_ros2) for an equivalent ROS2 driver.
# Getting Started

See the full instructions at Lucid:  https://support.thinklucid.com/using-ros-for-linux/

This package builds in the same way as other `catkin` ROS packages, however the [Arena SDK](https://thinklucid.com/downloads-hub/) must be installed.  See the [documentation at LucidVision](https://support.thinklucid.com/using-ros-for-linux/) for additional details.


# ROS Packages

This repo contains two ROS packages.  See the individual READMEs for each for details.

## camera_control_msgs

Contains actions and service calls for adjusting camera settings (gain, etc).

## arena_camera

A "generic" ROS node for Lucid cameras.  See the [README](arena_camera/README) in this package for more details.

# License

This code is forked from the [Lucid Vision's ROS node](https://github.com/lucidvisionlabs/arena_camera_ros), which in turn is forked from [Basler's ROS node for their Pylon library](https://github.com/magazino/pylon_camera), which is released under the [BSD 3-Clause License](LICENSE).
