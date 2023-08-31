/******************************************************************************
 * Software License Agreement (BSD License)
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

#include <pluginlib/class_list_macros.h>

#include <memory>

#include "arena_camera/arena_camera_nodelet.h"

namespace arena_camera {

using sensor_msgs::CameraInfo;
using sensor_msgs::CameraInfoPtr;

ArenaCameraPolledNodelet::ArenaCameraPolledNodelet()
    : ArenaCameraNodeletBase(), grab_imgs_raw_as_() {}

ArenaCameraPolledNodelet::~ArenaCameraPolledNodelet() {}

//
// Nodelet::onInit  function

void ArenaCameraPolledNodelet::onSuccessfulInit() {
  try {
    pDevice_->StartStream();
  } catch (GenICam::GenericException &e) {
    NODELET_ERROR_STREAM("Error while configuring camera: \r\n"
                         << e.GetDescription());
    return;
  }

  grab_imgs_raw_as_.reset(new GrabImagesAS(
      getNodeHandle(), "grab_images_raw",
      boost::bind(&ArenaCameraPolledNodelet::grabImagesRawActionExecuteCB, this,
                  _1),
      false));

  grab_imgs_raw_as_->start();
}

bool ArenaCameraPolledNodelet::sendSoftwareTrigger() {
  boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);
  bool retval = false;

  try {
    GenApi::CStringPtr pTriggerMode =
        pDevice_->GetNodeMap()->GetNode("TriggerMode");

    if (GenApi::IsWritable(pTriggerMode)) {
      bool isTriggerArmed = false;

      do {
        isTriggerArmed =
            Arena::GetNodeValue<bool>(pDevice_->GetNodeMap(), "TriggerArmed");
      } while (isTriggerArmed == false);
      Arena::ExecuteNode(pDevice_->GetNodeMap(), "TriggerSoftware");
    }
  } catch (GenICam::GenericException &e) {
    ;
  }

  return true;
}

bool ArenaCameraPolledNodelet::grabImage() {
  boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);
  bool retval = false;

  try {
    GenApi::CStringPtr pTriggerMode =
        pDevice_->GetNodeMap()->GetNode("TriggerMode");

    if (GenApi::IsWritable(pTriggerMode)) {
      bool isTriggerArmed = false;

      do {
        isTriggerArmed =
            Arena::GetNodeValue<bool>(pDevice_->GetNodeMap(), "TriggerArmed");
      } while (isTriggerArmed == false);
      Arena::ExecuteNode(pDevice_->GetNodeMap(), "TriggerSoftware");
    }

    Arena::IBuffer *pBuffer = pDevice_->GetBuffer(250);

    if (pBuffer->HasImageData()) {
      Arena::IImage *pImage = dynamic_cast<Arena::IImage *>(pBuffer);

      if (pImage->IsIncomplete()) {
        auto node_map = pDevice_->GetNodeMap();
        const auto payload_size =
            Arena::GetNodeValue<int64_t>(node_map, "PayloadSize");
        if (pBuffer->DataLargerThanBuffer()) {
          NODELET_WARN_STREAM("Image incomplete: data larger than buffer;  "
                              << pBuffer->GetSizeOfBuffer() << " > "
                              << payload_size);
        } else {
          NODELET_WARN_STREAM("Image incomplete: Payload size = "
                              << pBuffer->GetSizeFilled() << " , buffer size = "
                              << pBuffer->GetSizeOfBuffer() << " , expected "
                              << payload_size);
        }

        goto out;
      }

      sensor_msgs::Image::Ptr img_msg(new sensor_msgs::Image());

      img_msg->header.stamp = ros::Time::now();
      img_msg->header.frame_id = arena_camera_parameter_set_.cameraFrame();

      // Will return false if PixelEndiannessUnknown
      img_msg->is_bigendian =
          (pImage->GetPixelEndianness() == Arena::PixelEndiannessBig);

      img_msg->encoding = currentImageEncoding();
      img_msg->height = pImage->GetHeight();
      img_msg->width = pImage->GetWidth();

      const unsigned int bytes_per_pixel = pImage->GetBitsPerPixel() / 8;
      img_msg->step = img_msg->width * bytes_per_pixel;

      const unsigned int data_size = img_msg->height * img_msg->step;

      // \todo{amarburg} Compare to Buffer/Image payload size

      img_msg->data.resize(data_size);
      memcpy(&img_msg->data[0], pImage->GetData(), data_size);

      retval = true;
    }

  out:
    pDevice_->RequeueBuffer(pBuffer);
  } catch (GenICam::GenericException &e) {
    ;
  }

  return retval;
}

void ArenaCameraPolledNodelet::grabImagesRawActionExecuteCB(
    const camera_control_msgs::GrabImagesGoal::ConstPtr &goal) {
  camera_control_msgs::GrabImagesResult result;
  result = grabImagesRaw(goal, grab_imgs_raw_as_.get());
  grab_imgs_raw_as_->setSucceeded(result);
}

camera_control_msgs::GrabImagesResult ArenaCameraPolledNodelet::grabImagesRaw(
    const camera_control_msgs::GrabImagesGoal::ConstPtr &goal,
    GrabImagesAS *action_server) {
  camera_control_msgs::GrabImagesResult result;
  camera_control_msgs::GrabImagesFeedback feedback;

#if DEBUG
  std::cout << *goal << std::endl;
#endif

  // if (goal->exposure_given && goal->exposure_times.empty()) {
  //   NODELET_ERROR_STREAM(
  //       "GrabImagesRaw action server received request and "
  //       << "'exposure_given' is true, but the 'exposure_times' vector is "
  //       << "empty! Not enough information to execute acquisition!");
  //   result.success = false;
  //   return result;
  // }

  if (goal->gain_given && goal->gain_values.empty()) {
    NODELET_ERROR_STREAM(
        "GrabImagesRaw action server received request and "
        << "'gain_given' is true, but the 'gain_values' vector is "
        << "empty! Not enough information to execute acquisition!");
    result.success = false;
    return result;
  }

  if (goal->brightness_given && goal->brightness_values.empty()) {
    NODELET_ERROR_STREAM(
        "GrabImagesRaw action server received request and "
        << "'brightness_given' is true, but the 'brightness_values' vector"
        << " is empty! Not enough information to execute acquisition!");
    result.success = false;
    return result;
  }

  if (goal->gamma_given && goal->gamma_values.empty()) {
    NODELET_ERROR_STREAM(
        "GrabImagesRaw action server received request and "
        << "'gamma_given' is true, but the 'gamma_values' vector is "
        << "empty! Not enough information to execute acquisition!");
    result.success = false;
    return result;
  }

  std::vector<size_t> candidates;
  candidates.resize(4);  // gain, exposure, gamma, brightness
  candidates.at(0) = goal->gain_given ? goal->gain_values.size() : 0;
  candidates.at(1) = goal->exposure_given ? goal->exposure_times.size() : 0;
  candidates.at(2) =
      goal->brightness_given ? goal->brightness_values.size() : 0;
  candidates.at(3) = goal->gamma_given ? goal->gamma_values.size() : 0;

  size_t n_images = *std::max_element(candidates.begin(), candidates.end());

  if (goal->exposure_given && goal->exposure_times.size() != n_images) {
    NODELET_ERROR_STREAM(
        "Size of requested exposure times does not match to "
        << "the size of the requested vaules of brightness, gain or "
        << "gamma! Can't grab!");
    result.success = false;
    return result;
  }

  if (goal->gain_given && goal->gain_values.size() != n_images) {
    NODELET_ERROR_STREAM(
        "Size of requested gain values does not match to "
        << "the size of the requested exposure times or the vaules of "
        << "brightness or gamma! Can't grab!");
    result.success = false;
    return result;
  }

  if (goal->gamma_given && goal->gamma_values.size() != n_images) {
    NODELET_ERROR_STREAM(
        "Size of requested gamma values does not match to "
        << "the size of the requested exposure times or the vaules of "
        << "brightness or gain! Can't grab!");
    result.success = false;
    return result;
  }

  if (goal->brightness_given && goal->brightness_values.size() != n_images) {
    NODELET_ERROR_STREAM(
        "Size of requested brightness values does not match to "
        << "the size of the requested exposure times or the vaules of gain "
           "or "
        << "gamma! Can't grab!");
    result.success = false;
    return result;
  }

  if (goal->brightness_given && !(goal->exposure_auto || goal->gain_auto)) {
    NODELET_ERROR_STREAM(
        "Error while executing the GrabImagesRawAction: A "
        << "target brightness is provided but Exposure time AND gain are "
        << "declared as fix, so its impossible to reach the brightness");
    result.success = false;
    return result;
  }

  result.images.resize(n_images);
  result.reached_exposure_times.resize(n_images);
  result.reached_gain_values.resize(n_images);
  result.reached_gamma_values.resize(n_images);
  result.reached_brightness_values.resize(n_images);

  result.success = true;

  boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);

  float previous_exp, previous_gain, previous_gamma;
  if (goal->exposure_given) {
    previous_exp =
        Arena::GetNodeValue<double>(pDevice_->GetNodeMap(), "ExposureTime");
  }
  if (goal->gain_given) {
    previous_gain = currentGain();
  }
  if (goal->gamma_given) {
    previous_gamma = currentGamma();
  }
  if (goal->brightness_given) {
    previous_gain = currentGain();
    previous_exp = currentExposure();
  }

  for (std::size_t i = 0; i < n_images; ++i) {
    if (goal->exposure_given) {
      // result.success = setExposure(goal->exposure_times[i],
      //                              result.reached_exposure_times[i]);
    }
    if (goal->gain_given) {
      result.success = setGain(ArenaCameraPolledNodelet::AutoGainMode::Off,
                               goal->gain_values[i]);
      result.reached_gain_values[i] = currentGain();
    }
    if (goal->gamma_given) {
      result.success = setGamma(goal->gamma_values[i]);
      result.reached_gamma_values[i] = currentGamma();
    }
    if (goal->brightness_given) {
      // int reached_brightness;
      // result.success =
      //     setBrightness(goal->brightness_values[i], reached_brightness,
      //                   goal->exposure_auto, goal->gain_auto);
      // result.reached_brightness_values[i] =
      //     static_cast<float>(reached_brightness);
      //    result.reached_exposure_times[i] = currentExposure();
      //    result.reached_gain_values[i] = currentGain();
    }
    if (!result.success) {
      NODELET_ERROR_STREAM(
          "Error while setting one of the desired image "
          << "properties in the GrabImagesRawActionCB. Aborting!");
      break;
    }

    // \todo{amarburg}  What is this?
    // sensor_msgs::Image &img = result.images[i];
    // img.encoding = currentROSEncoding();
    // img.height = pImage_->GetHeight();
    // img.width = pImage_->GetWidth();
    // // step = full row length in bytes, img_size = (step * rows),
    // // imagePixelDepth already contains the number of channels
    // img_msg->step = img_msg->width * (pImage_->GetBitsPerPixel() /
    // 8);

    // img.header.stamp = ros::Time::now();
    // img.header.frame_id = cameraFrame();
    feedback.curr_nr_images_taken = i + 1;

    if (action_server != nullptr) {
      action_server->publishFeedback(feedback);
    }
  }
  if (camera_info_manager_) {
    sensor_msgs::CameraInfoPtr cam_info(
        new sensor_msgs::CameraInfo(camera_info_manager_->getCameraInfo()));
    result.cam_info = *cam_info;
  }

  // restore previous settings:
  float reached_val;
  if (goal->exposure_given) {
    // setExposure(previous_exp, reached_val);
  }
  if (goal->gain_given) {
    // setGain(previous_gain, reached_val);
  }
  if (goal->gamma_given) {
    // setGamma(previous_gamma, reached_val);
  }
  if (goal->brightness_given) {
    // setGain(previous_gain, reached_val);
    //  setExposure(previous_exp, reached_val);
  }
  return result;
}

}  // namespace arena_camera

PLUGINLIB_EXPORT_CLASS(arena_camera::ArenaCameraPolledNodelet, nodelet::Nodelet)
