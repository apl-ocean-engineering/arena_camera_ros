/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2023 University of Washington. All rights reserved.
 *
 * based on, with original license that follows
 *
 * Copyright (C) 2016, Magazino GmbH. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Magazino GmbH nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#pragma once

#include <arena_camera/ArenaCameraConfig.h>
#include <ros/ros.h>

#include <string>
#include <vector>

namespace arena_camera {
enum SHUTTER_MODE {
  SM_ROLLING = 0,
  SM_GLOBAL = 1,
  SM_GLOBAL_RESET_RELEASE = 2,
  SM_DEFAULT = -1,
};

/**
 *Parameter class for the ArenaCamera
 */
class ArenaCameraParameter {
 public:
  ArenaCameraParameter();

  virtual ~ArenaCameraParameter();

  /**
   * Read the parameters from the parameter server.
   * If invalid parameters can be detected, the interface will reset them
   * to the default values.
   * @param pnh the **private** ros::NodeHandle to use
   */
  void readFromRosParameterServer(const ros::NodeHandle &pnh);

  /// Getter for the device_user_id param
  const std::string &deviceUserID() const { return device_user_id_; }

  /// Getter for the serial number param
  const std::string &serialNumber() const { return serial_number_; }

  int mtuSize() const { return mtu_size_; }

  /**
   * Getter for the string describing the shutter mode
   */
  std::string shutterModeString() const;

  // Getter for the camera_frame_ set from ros-parameter server
  const std::string &cameraFrame() const { return camera_frame_; }

  // Getter for the image_encoding_ read from ros-parameter server
  const std::string &imageEncoding() const { return image_encoding_; }

  // Getter for the camera_info_url set from ros-parameter server
  const std::string &cameraInfoURL() const { return camera_info_url_; }

  /**
   * Setter for the camera_info_url_ if a new CameraInfo-Msgs Object is
   * provided via the SetCameraInfo-service from the CameraInfoManager
   */
  void setCameraInfoURL(const ros::NodeHandle &nh,
                        const std::string &camera_info_url);

 public:
  /** Binning factor to get downsampled images. It refers here to any camera
   * setting which combines rectangular neighborhoods of pixels into larger
   * "super-pixels." It reduces the resolution of the output image to
   * (width / binning_x) x (height / binning_y).
   * The default values binning_x = binning_y = 0 are considered the same
   * as binning_x = binning_y = 1 (no subsampling).
   */
  size_t binning_x_;
  size_t binning_y_;

  /**
   * Flags which indicate if the binning factors are provided and hence
   * should be set during startup
   */
  bool binning_x_given_;
  bool binning_y_given_;

  /**
   * Factor that describes the image downsampling to speed up the exposure
   * search to find the desired brightness.
   * The smallest window height is img_rows/downsampling_factor
   */
  int downsampling_factor_exp_search_;

  // #######################################################################
  // ###################### Image Intensity Settings  ######################
  // #######################################################################

  bool enable_lut_;

  // #######################################################################

  /**
   * The MTU size. Only used for GigE cameras.
   * To prevent lost frames the camera has to be configured
   * with the MTU size the network card supports. A value greater 3000
   * should be good (1500 for RaspberryPI)
   */
  int mtu_size_;

  /**
   * The inter-package delay in ticks. Only used for GigE cameras.
   * To prevent lost frames it should be greater 0.
   * For most of GigE-Cameras, a value of 1000 is reasonable.
   * For GigE-Cameras used on a RaspberryPI this value should be set to 11772
   */
  int inter_pkg_delay_;

  /**
   Shutter mode
  */
  SHUTTER_MODE shutter_mode_;

  /**
   * Flag that indicates if the camera has been calibrated and the intrinsic
   * calibration matrices are available
   */
  bool has_intrinsic_calib_;

  /**
   * Flag that indicates if the camera has a flash connected which should be on
   * on exposure Only supported for GigE cameras. Default: false
   */
  bool auto_flash_;
  /**
   * Flag that indicates if the camera, when using auto_flash == true, a flash
   * connected on line 2 which should be on on exposure Only supported for GigE
   * cameras. Default: true
   */
  bool auto_flash_line_2_;
  /**
   * Flag that indicates if the camera has, when using auto_flash == true,  a
   * flash connected on line 3 which should be on on exposure Only supported for
   * GigE cameras. Default: true
   */
  bool auto_flash_line_3_;

 protected:
  /**
   * Validates the parameter set found on the ros parameter server.
   * If invalid parameters can be detected, the interface will reset them
   * to the default values.
   * @param nh the ros::NodeHandle to use
   */
  void validateParameterSet(const ros::NodeHandle &nh);

  /**
   * The tf frame under which the images were published
   */
  std::string camera_frame_;

  /**
   * The DeviceUserID of the camera. If empty, the first camera found in the
   * device list will be used
   */
  std::string device_user_id_;

  std::string serial_number_;

  /**
   * The CameraInfo URL (Uniform Resource Locator) where the optional
   * intrinsic camera calibration parameters are stored. This URL string will
   * be parsed from the CameraInfoManager:
   * http://wiki.ros.org/camera_info_manager
   */
  std::string camera_info_url_;

  /**
   * The encoding of the pixels -- channel meaning, ordering, size taken
   * from the list of strings in include/sensor_msgs/image_encodings.h
   * The supported encodings are 'mono8', 'bgr8', 'rgb8', 'bayer_bggr8',
   * 'bayer_gbrg8', 'bayer_rggb8' and 'yuv422'
   */
  std::string image_encoding_;
};

}  // namespace arena_camera
