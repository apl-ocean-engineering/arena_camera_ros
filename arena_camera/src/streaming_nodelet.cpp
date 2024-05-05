/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2023, University of Washington. All rights reserved.
 *
 * based on
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
 *   * Neither the names of of the copyright holders nor the names of its
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

#include "arena_camera/arena_camera_nodelet.h"
#include "arena_camera/encoding_conversions.h"

namespace arena_camera {

using sensor_msgs::CameraInfo;
using sensor_msgs::CameraInfoPtr;

ArenaCameraStreamingNodelet::ArenaCameraStreamingNodelet()
    : ArenaCameraNodeletBase(),
      image_callback_obj_(std::bind(&ArenaCameraStreamingNodelet::imageCallback,
                                    this, std::placeholders::_1)) {}

ArenaCameraStreamingNodelet::~ArenaCameraStreamingNodelet() {
  pDevice_->DeregisterImageCallback(&image_callback_obj_);
}

//
// Nodelet::onInit  function

void ArenaCameraStreamingNodelet::onCameraConnect() {
  try {
    startStreaming();
  } catch (GenICam::GenericException &e) {
    NODELET_ERROR_STREAM("Error while configuring camera: \r\n"
                         << e.GetDescription());
    return;
  }

  pDevice_->RegisterImageCallback(&image_callback_obj_);
}

void ArenaCameraStreamingNodelet::imageCallback(Arena::IImage *pImage) {
  // Is this always true if the ImageCallback is called?
  Arena::IBuffer *pBuffer = dynamic_cast<Arena::IBuffer *>(pImage);
  if (pBuffer->HasImageData()) {
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
                            << pBuffer->GetSizeFilled()
                            << " , buffer size = " << pBuffer->GetSizeOfBuffer()
                            << " , expected " << payload_size);
      }

      return;
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

    // NODELET_INFO_STREAM("Image size " << pImage->GetWidth() << " x " <<
    // pImage->GetHeight() << " with " << pImage->GetBitsPerPixel() << " bits");
    // NODELET_INFO_STREAM("  expected size "
    //                     << (pImage->GetWidth() * pImage->GetHeight() *
    //                         (pImage->GetBitsPerPixel() / 8))
    //                     << " ; Image size " << data_size << " ; size filled "
    //                     << pImage->GetSizeFilled());

    // \todo{amarburg} Validate image by comparing calculated image
    //  size to actual Buffer/Image payload size
    img_msg->data.resize(data_size);
    memcpy(&img_msg->data[0], pImage->GetData(), data_size);

    if (img_raw_pub_.getNumSubscribers() > 0) {
      // Create a new cam_info-object in every frame, because it might have
      // changed due to a 'set_camera_info'-service call
      sensor_msgs::CameraInfo::Ptr cam_info(
          new sensor_msgs::CameraInfo(camera_info_manager_->getCameraInfo()));
      cam_info->header.stamp = img_msg->header.stamp;
      cam_info->header.frame_id = img_msg->header.frame_id;

      img_raw_pub_.publish(img_msg, cam_info);
    }

    imaging_msgs::ImagingMetadata meta_msg;
    meta_msg.header = img_msg->header;

    meta_msg.exposure_us = currentExposure();
    meta_msg.gain = currentGain();

    metadata_pub_.publish(meta_msg);

    if (encoding_conversions::isHDR(currentImageEncoding())) {
      imaging_msgs::HdrImagingMetadata hdr_meta_msg;
      hdr_meta_msg.header = meta_msg.header;
      hdr_meta_msg.exposure_us = meta_msg.exposure_us;
      hdr_meta_msg.gain = meta_msg.gain;

      const int num_hdr_channels = 4;
      hdr_meta_msg.hdr_exposure_us.resize(num_hdr_channels);
      hdr_meta_msg.hdr_gain.resize(num_hdr_channels);

      for (int hdr_channel = 0; hdr_channel < num_hdr_channels; hdr_channel++) {
        hdr_meta_msg.hdr_exposure_us[hdr_channel] =
            currentHdrExposure(hdr_channel);
        hdr_meta_msg.hdr_gain[hdr_channel] = currentHdrGain(hdr_channel);
      }

      hdr_metadata_pub_.publish(hdr_meta_msg);
    }
  }
}

void ArenaCameraStreamingNodelet::reconfigureCallback(ArenaCameraConfig &config,
                                                      uint32_t level) {
  stopStreaming();

  ArenaCameraNodeletBase::reconfigureCallback(config, level);

  startStreaming();
}

}  // namespace arena_camera

PLUGINLIB_EXPORT_CLASS(arena_camera::ArenaCameraStreamingNodelet,
                       nodelet::Nodelet)
