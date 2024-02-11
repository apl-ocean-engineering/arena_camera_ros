/******************************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 * Copyright (C) 2023 University of Washington. All rights reserved.
 *
 * Based on the original arena_camera_ros as follows:
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

// STD
#include <memory>
#include <string>

// ROS sys dep
#include <boost/thread.hpp>

// ROS
#include <actionlib/server/simple_action_server.h>
#include <camera_control_msgs/GrabImagesAction.h>
#include <camera_control_msgs/SetBinning.h>
#include <camera_control_msgs/SetBool.h>
#include <camera_control_msgs/SetBrightness.h>
#include <camera_control_msgs/SetExposure.h>
#include <camera_control_msgs/SetGain.h>
#include <camera_control_msgs/SetGamma.h>
#include <camera_control_msgs/SetROI.h>
#include <camera_info_manager/camera_info_manager.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

// Arena
#include <ArenaApi.h>
#include <GenApi/GenApi.h>
#include <arena_camera/arena_camera_parameter.h>

// Auto-generated dynamic_reconfigure header file
#include <arena_camera/ArenaCameraConfig.h>

#include "imaging_msgs/HdrImagingMetadata.h"
#include "imaging_msgs/ImagingMetadata.h"

namespace arena_camera {
typedef actionlib::SimpleActionServer<camera_control_msgs::GrabImagesAction>
    GrabImagesAS;

/// Base class for both types of nodelets
class ArenaCameraNodeletBase : public nodelet::Nodelet {
 public:
  ArenaCameraNodeletBase();
  virtual ~ArenaCameraNodeletBase();

  /**
   * initialize the camera and the ros node.
   * calls ros::shutdown if an error occurs.
   */
  void onInit() override;

  /// Getter for the tf frame.
  /// @return the camera frame.
  ///
  const std::string &cameraFrame() const {
    return arena_camera_parameter_set_.cameraFrame();
  }

  void startStreaming();
  void stopStreaming();

 protected:
  /**
   * Creates the camera instance either by UserDeviceId, SerialNumber,
   * or taking the first auto-detected camera.
   * @return false if an error occurred
   */
  bool registerCameraByUserId(const std::string &device_id);
  bool registerCameraBySerialNumber(const std::string &serial_number);
  bool registerCameraByAuto();

  ///
  /// Start the camera and initialize the messages
  /// @return
  ///
  bool configureCamera();

  /// Virtual callback for node initialization _after_ Node::onInit()
  /// Only called if that initialization/configuration is successful.
  virtual void onSuccessfulInit(){};

  // === Functions to set/get ImageEncoding ===

  bool setImageEncoding(const std::string &ros_encoding);

  std::string currentImageEncoding();

  // === Functions to get/set Frame Rate ===

  float currentFrameRate();
  void updateFrameRate(float frame_rate);

  //==== Functions to get/set exposure ====

  enum class AutoExposureMode : int { Off = 0, Once = 1, Continuous = 2 };

  /// Update exposure based on arena_camera_parameter_set
  ///
  ///  If exp_mode == Off, exposure_ms is the **fixed exposure** set in the
  ///  camera
  ///
  ///  If exp_mode == Once or Continuous, exposure_ms is the **max
  ///  exposure** allowed
  ///                   for the auto-exposure algorithm
  void setExposure(AutoExposureMode exp_mode, float exposure_ms);

  void setAutoExposureGain(float exposure_damping);

  /// Return current camera ExposureTime
  /// @param immediate If true, will directly query the camera. Otherwise uses a
  /// cached value (updated by callback)
  /// @return  Camera ExposureTime node (in us)
  float currentExposure(bool immediate = false);

  //==== Functions to get/set gain ====

  enum class AutoGainMode : int { Off = 0, Once = 1, Continuous = 2 };

  /**
   * Update the gain from the camera to a target gain in percent
   * @param gain_mode   Request gain mode
   * @param target_gain the targeted gain in percent.  Ignored if gain_mode
   * isn't "Off"
   * @param reached_gain the gain that could be reached
   * @return true if the targeted gain could be reached
   */
  bool setGain(AutoGainMode gain_mode, float target_gain = 0.0);

  /// Return current camera Gain
  /// @param immediate If true, will directly query the camera. Otherwise uses a
  /// cached value (updated by callback)
  /// @return  Camera Gain node
  float currentGain(bool immediate = false);

  //==== Functions to get/set gamma ====

  /**
   * Update the gamma from the camera to a target gamma correction value
   * @param target_gamma the targeted gamma
   * @return true if the targeted gamma could be reached
   */
  bool setGamma(const float &target_gamma);
  float currentGamma();

  //===== Functions for querying HDR channels (IMX490 only)
  float currentHdrGain(int channel);
  float currentHdrExposure(int channel);

  // === Functions to get/set ROI ===

  /**
   * Update area of interest in the camera image
   * @param target_roi the target roi
   * @param reached_roi the roi that could be set
   * @return true if the targeted roi could be reached
   */
  bool setROI(const sensor_msgs::RegionOfInterest target_roi,
              sensor_msgs::RegionOfInterest &reached_roi);

  // === Functions to get/set ROI ===

  /**
   * Update the horizontal binning_x factor to get downsampled images
   * @param target_binning_x the target horizontal binning_x factor
   * @param reached_binning_x the horizontal binning_x factor that could be
   *        reached
   * @return true if the targeted binning could be reached
   */
  bool setBinningX(const size_t &target_binning_x, size_t &reached_binning_x);

  /**
   * Update the vertical binning_y factor to get downsampled images
   * @param target_binning_y the target vertical binning_y factor
   * @param reached_binning_y the vertical binning_y factor that could be
   *        reached
   * @return true if the targeted binning could be reached
   */
  bool setBinningY(const size_t &target_binning_y, size_t &reached_binning_y);

  /**
   * Sets the target brightness which is the intensity-mean over all pixels.
   */

  void setTargetBrightness(unsigned int brightness);

  /**
   *  Enable/disable lookup table (LUT) in camera.
   * @param enable Whether to enable/disable the camera LUT
   */
  void enableLUT(bool enable);

 protected:
  Arena::ISystem *pSystem_;
  Arena::IDevice *pDevice_;

  bool is_streaming_;

  // Hardware accessor functions
  // These might have originally been in arena_camera.h?
  sensor_msgs::RegionOfInterest currentROI();
  int64_t currentBinningX();
  int64_t currentBinningY();
  bool setBinningXValue(const size_t &target_binning_x,
                        size_t &reached_binning_x);
  bool setBinningYValue(const size_t &target_binning_y,
                        size_t &reached_binning_y);

  ros::Publisher metadata_pub_, hdr_metadata_pub_;

  ArenaCameraParameter arena_camera_parameter_set_;

  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher img_raw_pub_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

  std::vector<std::size_t> sampling_indices_;

  boost::recursive_mutex device_mutex_;

  // Internal cache for exposure and gain, updated by callback
  float cached_exposure_, cached_gain_;
  GenApi::CallbackHandleType exposure_changed_callback_, gain_changed_callback_;

  void onExposureChangeCallback(GenApi::INode *pNode);
  void onGainChangeCallback(GenApi::INode *pNode);

  ros::Timer camera_poll_timer_;

  typedef dynamic_reconfigure::Server<arena_camera::ArenaCameraConfig>
      DynReconfigureServer;
  std::shared_ptr<DynReconfigureServer> _dynReconfigureServer;

  // Non-virtual callback which calls virtual function
  void reconfigureCallbackWrapper(ArenaCameraConfig &config, uint32_t level) {
    reconfigureCallback(config, level);
  }

  virtual void reconfigureCallback(ArenaCameraConfig &config, uint32_t level);
  ArenaCameraConfig previous_config_;

  /// diagnostics:
  diagnostic_updater::Updater diagnostics_updater_;
  ros::Timer diagnostics_trigger_;
  void create_camera_info_diagnostics(
      diagnostic_updater::DiagnosticStatusWrapper &stat);
};

class ArenaCameraStreamingNodelet : public ArenaCameraNodeletBase {
 public:
  ArenaCameraStreamingNodelet();
  virtual ~ArenaCameraStreamingNodelet();

  void onSuccessfulInit() override;

 protected:
  typedef std::function<void(Arena::IImage *pImage)> ImageCallback_t;

  class ImageCallback : public Arena::IImageCallback {
   public:
    ImageCallback(ImageCallback_t cb) : image_callback_(cb) {}
    ~ImageCallback() {}

    void OnImage(Arena::IImage *pImage) { image_callback_(pImage); }

   private:
    ImageCallback_t image_callback_;

    ImageCallback() = delete;
    ImageCallback(const ImageCallback &) = delete;
    ImageCallback operator=(const ImageCallback &) = delete;
  } image_callback_obj_;

  void imageCallback(Arena::IImage *pImage);

  void reconfigureCallback(ArenaCameraConfig &config, uint32_t level) override;
};

class ArenaCameraPolledNodelet : public ArenaCameraNodeletBase {
 public:
  ArenaCameraPolledNodelet();
  virtual ~ArenaCameraPolledNodelet();

  void onSuccessfulInit() override;

  /// Callback for the grab images action
  /// @param goal the goal
  ///
  void grabImagesRawActionExecuteCB(
      const camera_control_msgs::GrabImagesGoal::ConstPtr &goal);

  /// This function can also be called from the derived ArenaCameraOpenCV-Class
  ///
  camera_control_msgs::GrabImagesResult grabImagesRaw(
      const camera_control_msgs::GrabImagesGoal::ConstPtr &goal,
      GrabImagesAS *action_server);

 protected:
  virtual bool sendSoftwareTrigger();

  /// Grabs an image and stores the image in img_raw_msg_
  /// @return false if an error occurred.
  virtual bool grabImage();

  std::unique_ptr<GrabImagesAS> grab_imgs_raw_as_;

  void reconfigureCallback(ArenaCameraConfig &config, uint32_t level) override {
    ;
  }
};

}  // namespace arena_camera
