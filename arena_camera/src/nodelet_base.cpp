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

// STD
#include <algorithm>
#include <boost/multi_array.hpp>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

// ROS
#include <dynamic_reconfigure/SensorLevels.h>
#include <sensor_msgs/RegionOfInterest.h>

// Arena
#include <ArenaApi.h>
#include <GenApi/GenApi.h>
#include <GenApiCustom.h>

// Arena node
#include "arena_camera/arena_camera_nodelet.h"
#include "arena_camera/encoding_conversions.h"

using diagnostic_msgs::DiagnosticStatus;

namespace arena_camera {

using sensor_msgs::CameraInfo;
using sensor_msgs::CameraInfoPtr;

ArenaCameraNodeletBase::ArenaCameraNodeletBase()
    : pSystem_(nullptr),
      pDevice_(nullptr),
      pNodeMap_(nullptr),
      is_streaming_(false),
      arena_camera_parameter_set_(),
      set_user_output_srvs_(),
      it_(nullptr),
      img_raw_pub_(),
      pinhole_model_(),
      camera_info_manager_(nullptr),
      sampling_indices_() {}

ArenaCameraNodeletBase::~ArenaCameraNodeletBase() {
  if (pDevice_ != nullptr) {
    pSystem_->DestroyDevice(pDevice_);
  }

  if (pSystem_ != nullptr) {
    Arena::CloseSystem(pSystem_);
  }
}

//===================================================================
//
// onInit

void ArenaCameraNodeletBase::onInit() {
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &pnh = getPrivateNodeHandle();

  metadata_pub_ =
      nh.advertise<imaging_msgs::ImagingMetadata>("imaging_metadata", 1);

  it_.reset(new image_transport::ImageTransport(nh));
  img_raw_pub_ = it_->advertiseCamera("image_raw", 1);

  camera_info_manager_ = new camera_info_manager::CameraInfoManager(nh);

  diagnostics_updater_.setHardwareID("none");
  diagnostics_updater_.add("camera_availability", this,
                           &ArenaCameraNodeletBase::create_diagnostics);
  diagnostics_updater_.add(
      "intrinsic_calibration", this,
      &ArenaCameraNodeletBase::create_camera_info_diagnostics);
  diagnostics_trigger_ = nh.createTimer(
      ros::Duration(2), &ArenaCameraNodeletBase::diagnostics_timer_callback_,
      this);

  // Initialize parameters from parameter server
  arena_camera_parameter_set_.readFromRosParameterServer(pnh);

  // Open the Arena SDK
  pSystem_ = Arena::OpenSystem();
  pSystem_->UpdateDevices(100);
  if (pSystem_->GetDevices().size() == 0) {
    NODELET_FATAL("Did not detect any cameras!!");
    return;
  }

  if (!arena_camera_parameter_set_.deviceUserID().empty()) {
    if (!registerCameraByUserId(arena_camera_parameter_set_.deviceUserID())) {
      NODELET_FATAL_STREAM("Unable to find a camera with DeviceUserId \""
                           << arena_camera_parameter_set_.deviceUserID()
                           << "\"");
      return;
    }
  } else if (!arena_camera_parameter_set_.serialNumber().empty()) {
    if (!registerCameraBySerialNumber(
            arena_camera_parameter_set_.serialNumber())) {
      NODELET_FATAL_STREAM("Unable to find a camera with Serial Number "
                           << arena_camera_parameter_set_.serialNumber());
      return;
    }
  } else {
    if (!registerCameraByAuto()) {
      NODELET_FATAL_STREAM("Unable to find any cameras to register");
      return;
    }
  }

  // Validate that the camera is from Lucid (otherwise the Arena SDK
  // will segfault)
  const auto device_vendor_name = Arena::GetNodeValue<GenICam::gcstring>(
      pDevice_->GetNodeMap(), "DeviceVendorName");
  if (device_vendor_name != "Lucid Vision Labs") {
    NODELET_FATAL_STREAM(
        "Hm, this doesn't appear to be a Lucid Vision camera, got vendor name: "
        << device_vendor_name);
  }

  if (!configureCamera()) {
    NODELET_FATAL_STREAM("Unable to configure camera");
    return;
  }

  _dynReconfigureServer = std::make_shared<DynReconfigureServer>(pnh);
  _dynReconfigureServer->setCallback(boost::bind(
      &ArenaCameraNodeletBase::reconfigureCallbackWrapper, this, _1, _2));
}


//===================================================================
//
// Functions to find/register a camera

bool ArenaCameraNodeletBase::registerCameraByUserId(
    const std::string &device_user_id_to_open) {
  ROS_ASSERT(pSystem_);
  std::vector<Arena::DeviceInfo> deviceInfos = pSystem_->GetDevices();
  ROS_ASSERT(deviceInfos.size() > 0);

  NODELET_INFO_STREAM("Connecting to camera with DeviceUserId"
                      << device_user_id_to_open);

  std::vector<Arena::DeviceInfo>::iterator it;

  for (auto &dev : deviceInfos) {
    const std::string device_user_id(dev.UserDefinedName());
    // Must be an exact match
    if (0 == device_user_id_to_open.compare(device_user_id)) {
      NODELET_INFO_STREAM("Found the desired camera with DeviceUserID "
                          << device_user_id_to_open << ": ");

      pDevice_ = pSystem_->CreateDevice(dev);
      return true;
    }
  }

  NODELET_ERROR_STREAM(
      "Couldn't find the camera that matches the "
      << "given DeviceUserID: \"" << device_user_id_to_open << "\"! "
      << "Either the ID is wrong or the cam is not yet connected");
  return false;
}

bool ArenaCameraNodeletBase::registerCameraBySerialNumber(
    const std::string &serial_number) {
  ROS_ASSERT(pSystem_);
  std::vector<Arena::DeviceInfo> deviceInfos = pSystem_->GetDevices();
  ROS_ASSERT(deviceInfos.size() > 0);

  NODELET_INFO_STREAM("Connecting to camera with Serial Number "
                      << serial_number);

  for (auto &dev : deviceInfos) {
    if (0 == serial_number.compare(dev.SerialNumber())) {
      NODELET_INFO_STREAM("Found the desired camera with Serial Number "
                          << serial_number);

      pDevice_ = pSystem_->CreateDevice(dev);
      return true;
    }
  }

  NODELET_ERROR_STREAM(
      "Couldn't find the camera that matches the "
      << "given Serial Number: " << serial_number << "! "
      << "Either the ID is wrong or the cam is not yet connected");
  return false;
}

bool ArenaCameraNodeletBase::registerCameraByAuto() {
  ROS_ASSERT(pSystem_);
  std::vector<Arena::DeviceInfo> deviceInfos = pSystem_->GetDevices();
  ROS_ASSERT(deviceInfos.size() > 0);

  int i = 0;
  NODELET_INFO_STREAM("Found " << deviceInfos.size() << " cameras");
  for (auto &dev : deviceInfos) {
    NODELET_INFO_STREAM(i++ << ":  " << dev.SerialNumber() << "  "
                            << dev.UserDefinedName());
  }

  for (auto &dev : deviceInfos) {
    if (dev.VendorName() == "Lucid Vision Labs") {
      pDevice_ = pSystem_->CreateDevice(dev);
      NODELET_INFO_STREAM(
          "Connecting to first autodetected Lucid Vision camera: Serial Number "
          << dev.SerialNumber() << " ; User ID: " << dev.UserDefinedName());
      return true;
    }
  }

  return false;
}


//===================================================================
//
//

bool ArenaCameraNodeletBase::configureCamera() {
  ros::NodeHandle nh = getNodeHandle();
  auto pNodeMap = pDevice_->GetNodeMap();

  // **NOTE** only configuration which is not also accessible through
  // dynamic_reconfigure.  Those will be handled in the callback
  // when it is called for the first time at node startup.

  try {
    NODELET_INFO_STREAM(
        "   Device model: " << Arena::GetNodeValue<GenICam::gcstring>(
            pDevice_->GetNodeMap(), "DeviceModelName"));
    NODELET_INFO_STREAM(
        "Device firmware: " << Arena::GetNodeValue<GenICam::gcstring>(
            pDevice_->GetNodeMap(), "DeviceFirmwareVersion"));

    //
    // Arena device prior streaming settings
    //

    if (Arena::GetNodeValue<GenICam::gcstring>(
            pDevice_->GetNodeMap(), "DeviceTLType") == "GigEVision") {
      NODELET_INFO("GigE device, performing GigE specific configuration:");

      // Set Jumbo frames (this is only relevant for GigE cameras.)
      auto pPacketSize = pNodeMap->GetNode("DeviceStreamChannelPacketSize");
      if (GenApi::IsWritable(pPacketSize)) {
        NODELET_INFO_STREAM(" -> Setting MTU to "
                            << arena_camera_parameter_set_.mtuSize());
        Arena::SetNodeValue<int64_t>(pNodeMap, "DeviceStreamChannelPacketSize",
                                     arena_camera_parameter_set_.mtuSize());
      } else {
        NODELET_INFO(" -> Camera MTU is not writeable");
      }
    }

    auto payloadSize = Arena::GetNodeValue<int64_t>(pNodeMap, "PayloadSize");
    NODELET_INFO_STREAM("Expected payload size: " << payloadSize);

    // enable stream auto negotiate packet size
    NODELET_DEBUG("Enabling auto-negotiation of packet size");
    Arena::SetNodeValue<bool>(pDevice_->GetTLStreamNodeMap(),
                              "StreamAutoNegotiatePacketSize", true);

    // enable stream packet resend
    NODELET_DEBUG("Enabling packet resend");
    Arena::SetNodeValue<bool>(pDevice_->GetTLStreamNodeMap(),
                              "StreamPacketResendEnable", true);

    //
    // PIXELFORMAT
    //
    setImageEncoding(arena_camera_parameter_set_.imageEncoding());

    if (encoding_conversions::isHDR(
            arena_camera_parameter_set_.imageEncoding())) {
      hdr_metadata_pub_ = nh.advertise<imaging_msgs::HdrImagingMetadata>(
          "hdr_imaging_metadata", 1);
    }

    //
    // TRIGGER MODE
    //
    GenApi::CStringPtr pTriggerMode = pNodeMap->GetNode("TriggerMode");
    if (GenApi::IsWritable(pTriggerMode)) {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "TriggerMode", "On");
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "TriggerSource",
                                             "Software");
    }

    //!! Parameters controlled by param / dynamic reonfigure are not set here
    //!! Assume there will be an immediate call from dynamic reconfigure

    // LUT
    NODELET_INFO_STREAM(
        (arena_camera_parameter_set_.enable_lut_ ? "Enabling" : "Disabling")
        << " camera LUT");
    enableLUT(arena_camera_parameter_set_.enable_lut_);

    // ------------------------------------------------------------------------

    //
    //  Initial setting of the CameraInfo-msg, assuming no calibration given
    CameraInfo initial_cam_info;
    // initializeCameraInfo(initial_cam_info);
    camera_info_manager_->setCameraInfo(initial_cam_info);

    if (arena_camera_parameter_set_.cameraInfoURL().empty() ||
        !camera_info_manager_->validateURL(
            arena_camera_parameter_set_.cameraInfoURL())) {
      NODELET_INFO_STREAM("CameraInfoURL needed for rectification! ROS-Param: "
                          << "'" << nh.getNamespace() << "/camera_info_url' = '"
                          << arena_camera_parameter_set_.cameraInfoURL()
                          << "' is invalid!");
      NODELET_DEBUG_STREAM("CameraInfoURL should have following style: "
                           << "'file:///full/path/to/local/file.yaml' or "
                           << "'file://${ROS_HOME}/camera_info/${NAME}.yaml'");
      NODELET_WARN_STREAM("Will only provide distorted /image_raw images!");
    } else {
      // override initial camera info if the url is valid
      if (camera_info_manager_->loadCameraInfo(
              arena_camera_parameter_set_.cameraInfoURL())) {
        // setupRectification();
        // set the correct tf frame_id
        CameraInfoPtr cam_info(
            new CameraInfo(camera_info_manager_->getCameraInfo()));

      } else {
        NODELET_WARN_STREAM("Will only provide distorted /image_raw images!");
      }
    }

    if (arena_camera_parameter_set_.binning_x_given_) {
      size_t reached_binning_x;
      if (setBinningX(arena_camera_parameter_set_.binning_x_,
                      reached_binning_x)) {
        NODELET_INFO_STREAM("Setting horizontal binning_x to "
                            << arena_camera_parameter_set_.binning_x_);
        NODELET_WARN_STREAM(
            "The image width of the camera_info-msg will "
            << "be adapted, so that the binning_x value in this msg remains 1");
      }
    }

    if (arena_camera_parameter_set_.binning_y_given_) {
      size_t reached_binning_y;
      if (setBinningY(arena_camera_parameter_set_.binning_y_,
                      reached_binning_y)) {
        NODELET_INFO_STREAM("Setting vertical binning_y to "
                            << arena_camera_parameter_set_.binning_y_);
        NODELET_WARN_STREAM(
            "The image height of the camera_info-msg will "
            << "be adapted, so that the binning_y value in this msg remains 1");
      }
    }

    // if (arena_camera_parameter_set_.image_encoding_given_)
    // {
    // 	float reached_image_encoding;
    // 	if (setImageEncoding(arena_camera_parameter_set_.image_encoding_))
    // 	{
    // 		NODELET_INFO_STREAM("Setting encoding to "
    // 						<<
    // arena_camera_parameter_set_.image_encoding_);
    // 	}
    // }

    Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetTLStreamNodeMap(),
                                           "StreamBufferHandlingMode",
                                           "NewestOnly");

    // bool isTriggerArmed = false;

    // if (GenApi::IsWritable(pTriggerMode)) {
    //   do {
    //     isTriggerArmed = Arena::GetNodeValue<bool>(pNodeMap, "TriggerArmed");
    //   } while (isTriggerArmed == false);
    //   // Arena::ExecuteNode(pNodeMap, "TriggerSoftware");
    // }

    // pImage_ = pDevice_->GetImage(5000);
    // pData_ = pImage_->GetData();

    // img_raw_msg_.data.resize(img_raw_msg_.height * img_raw_msg_.step);
    // memcpy(&img_raw_msg_.data[0], pImage_->GetData(),
    //        img_raw_msg_.height * img_raw_msg_.step);
  } catch (GenICam::GenericException &e) {
    NODELET_ERROR_STREAM("Error while configuring camera: \r\n"
                         << e.GetDescription());
    return false;
  }

  // --------------------------------------------------------------------------
 
  if (!camera_info_manager_->setCameraName(std::string(
          Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "DeviceUserID")
              .c_str()))) {
    // valid name contains only alphanumeric signs and '_'
    NODELET_WARN_STREAM("["
                        << std::string(Arena::GetNodeValue<GenICam::gcstring>(
                                           pNodeMap, "DeviceUserID")
                                           .c_str())
                        << "] name not valid for camera_info_manager");
  }

  // NODELET_INFO("=== Startup settings ===");
  // NODELET_INFO_STREAM("encoding = " << currentImageEncoding());
  // NODELET_INFO_STREAM("binning = [" << currentBinningX() << " x "
  //                                   << currentBinningY() << "]");
  // NODELET_INFO_STREAM("exposure = " << currentExposure() << " us");
  // NODELET_INFO_STREAM("gain = " << currentGain());
  // NODELET_INFO_STREAM("gamma = " << currentGamma());
  // NODELET_INFO_STREAM(
  //     "shutter mode = " << arena_camera_parameter_set_.shutterModeString());
  // NODELET_INFO("========================");

  // pDevice_->RequeueBuffer(pImage_);
  return true;
}

//===================================================================
//
// Start/stop streaming

void ArenaCameraNodeletBase::startStreaming() {
  if (!is_streaming_) {
    pDevice_->StartStream();
    is_streaming_ = true;
  }
}

void ArenaCameraNodeletBase::stopStreaming() {
  if (is_streaming_) {
    pDevice_->StopStream();
    is_streaming_ = false;
  }
}

//===================================================================
//
// Get/set frame rate

// Note that streaming must be stopped before updating frame rate
void ArenaCameraNodeletBase::updateFrameRate() {
  try {
    ros::NodeHandle nh = getNodeHandle();
    auto pNodeMap = pDevice_->GetNodeMap();

    // const bool was_streaming = is_streaming_;
    // stopStreaming();

    auto cmdlnParamFrameRate = arena_camera_parameter_set_.frameRate();
    auto currentFrameRate =
        Arena::GetNodeValue<double>(pNodeMap, "AcquisitionFrameRate");
    auto maximumFrameRate =
        GenApi::CFloatPtr(pNodeMap->GetNode("AcquisitionFrameRate"))->GetMax();

    // requested framerate larger than device max so we trancate it
    if (cmdlnParamFrameRate >= maximumFrameRate) {
      arena_camera_parameter_set_.setFrameRate(maximumFrameRate);

      NODELET_WARN(
          "Desired framerate %.2f Hz (rounded) is higher than max possible. "
          "Will limit "
          "framerate device max : %.2f Hz (rounded)",
          cmdlnParamFrameRate, maximumFrameRate);
    }
    // special case:
    // dues to inacurate float comparision we skip. If we set it it might
    // throw becase it could be a lil larger than the max avoid the exception
    // (double accuracy issue when setting the node) request frame rate very
    // close to device max
    else if (cmdlnParamFrameRate == maximumFrameRate) {
      NODELET_INFO("Framerate is %.2f Hz", cmdlnParamFrameRate);
    }
    // requested max frame rate
    else if (cmdlnParamFrameRate ==
             -1)  // speacial for max frame rate available
    {
      arena_camera_parameter_set_.setFrameRate(maximumFrameRate);

      NODELET_WARN("Framerate is set to device max : %.2f Hz",
                   maximumFrameRate);
    }

    // requested framerate is valid so we set it to the device
    try {
      if (!Arena::GetNodeValue<bool>(pNodeMap, "AcquisitionFrameRateEnable")) {
        NODELET_INFO(
            "\"AcquisitionFrameRateEnable\" not true, trying to enable");
        Arena::SetNodeValue<bool>(pNodeMap, "AcquisitionFrameRateEnable", true);
      }
      Arena::SetNodeValue<double>(pNodeMap, "AcquisitionFrameRate",
                                  arena_camera_parameter_set_.frameRate());
    } catch (GenICam::GenericException &e) {
      NODELET_INFO_STREAM("Exception while changing frame rate: " << e.what());
    }
    NODELET_INFO_STREAM(
        "Framerate has been set to "
        << Arena::GetNodeValue<double>(pNodeMap, "AcquisitionFrameRate")
        << " Hz");

    // if (was_streaming) startStreaming();

  } catch (GenICam::GenericException &e) {
    NODELET_INFO_STREAM("Exception while changing frame rate: " << e.what());
  }
}

//===================================================================
//
// Get/set image encoding

std::string ArenaCameraNodeletBase::currentImageEncoding() {
  std::string gen_api_encoding(Arena::GetNodeValue<GenICam::gcstring>(
      pDevice_->GetNodeMap(), "PixelFormat"));
  std::string ros_encoding("");
  if (!encoding_conversions::genAPI2Ros(gen_api_encoding, ros_encoding)) {
    std::stringstream ss;
    ss << "No ROS equivalent to GenApi encoding '" << gen_api_encoding
       << "' found! This is bad because this case "
          "should never occur!";
    throw std::runtime_error(ss.str());
    return "NO_ENCODING";
  }
  return ros_encoding;
}

bool ArenaCameraNodeletBase::setImageEncoding(const std::string &ros_encoding) {
  std::string gen_api_encoding;
  bool conversion_found =
      encoding_conversions::ros2GenAPI(ros_encoding, gen_api_encoding);
  if (!conversion_found) {
    if (ros_encoding.empty()) {
      return false;
    } else {
      std::string fallbackPixelFormat =
          Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(),
                                                 "PixelFormat")
              .c_str();
      NODELET_ERROR_STREAM(
          "Can't convert ROS encoding '"
          << ros_encoding
          << "' to a corresponding GenAPI encoding! Will use current "
          << "pixel format ( " << fallbackPixelFormat << " ) as fallback!");
      return false;
    }
  }
  try {
    GenApi::CEnumerationPtr pPixelFormat =
        pDevice_->GetNodeMap()->GetNode("PixelFormat");
    if (GenApi::IsWritable(pPixelFormat)) {
      Arena::SetNodeValue<GenICam::gcstring>(
          pDevice_->GetNodeMap(), "PixelFormat", gen_api_encoding.c_str());
      if (currentImageEncoding() == "16UC3" || currentImageEncoding() == "16UC4")
        NODELET_WARN_STREAM(
            "ROS grabbing image data from 3D pixel format, unable to display "
            "in image viewer");
    }
  } catch (const GenICam::GenericException &e) {
    NODELET_ERROR_STREAM("An exception while setting target image encoding to '"
                         << ros_encoding
                         << "' occurred: " << e.GetDescription());
    return false;
  }

  NODELET_INFO("Have configured image_encoding");

  if (arena_camera::encoding_conversions::isHDR(ros_encoding)) {
    NODELET_INFO_STREAM("Requested HDR encoding \""
                        << ros_encoding << "\", enabling HDR mode in camera");

    try {
      auto pNodeMap = pDevice_->GetNodeMap();

      // GenApi::CStringPtr pHDROutput = pNodeMap->GetNode("HDROutput");
      // if (GenApi::IsWritable(pHDROutput)) {
      //   Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "HDROutput",
      //   "HDR");
      // }

      // Enable HDR image enhancement
      Arena::SetNodeValue<bool>(pNodeMap, "HDRImageEnhancementEnable", true);
      Arena::SetNodeValue<bool>(pNodeMap, "HDRTuningEnable", false);

      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "HDROutput", "HDR");

    } catch (GenICam::GenericException &e) {
      NODELET_ERROR_STREAM("Error while configuring camera: "
                           << e.GetDescription()
                           << ", is this camera capable of HDR?");
      return false;
    }
  }

  return true;
}

//===================================================================
//
// Get/set exposure


float ArenaCameraNodeletBase::currentExposure() {
  GenApi::CFloatPtr pExposureTime =
      pDevice_->GetNodeMap()->GetNode("ExposureTime");

  if (!pExposureTime || !GenApi::IsReadable(pExposureTime)) {
    NODELET_WARN_STREAM("No exposure time value, returning -1");
    return -1.;
  } else {
    float exposureValue = pExposureTime->GetValue();
    return exposureValue;
  }
}

void ArenaCameraNodeletBase::setExposure(
    ArenaCameraNodeletBase::AutoExposureMode exp_mode, float exposure_ms) {
  auto pNodeMap = pDevice_->GetNodeMap();
  // exposure_auto_ will be already set to false if exposure_given_ is true
  // read params () solved the priority between them
  if (exp_mode == ArenaCameraNodeletBase::AutoExposureMode::Off) {
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAuto", "Off");

    GenApi::CFloatPtr pExposureTime =
        pDevice_->GetNodeMap()->GetNode("ExposureTime");

    float exposure_to_set = exposure_ms * 1000;
    if (exposure_to_set < pExposureTime->GetMin()) {
      NODELET_WARN_STREAM("Desired exposure ("
                          << exposure_to_set << ") "
                          << "time unreachable! Setting to lower limit: "
                          << pExposureTime->GetMin());
      exposure_to_set = pExposureTime->GetMin();
    } else if (exposure_to_set > pExposureTime->GetMax()) {
      NODELET_WARN_STREAM("Desired exposure ("
                          << exposure_to_set << ") "
                          << "time unreachable! Setting to upper limit: "
                          << pExposureTime->GetMax());
      exposure_to_set = pExposureTime->GetMax();
    }

    pExposureTime->SetValue(exposure_to_set);

    NODELET_INFO_STREAM("Setting auto-exposure _off_ with exposure of "
                        << pExposureTime->GetValue() << " ms");
  } else {
    if (exp_mode == ArenaCameraNodeletBase::AutoExposureMode::Once) {
      NODELET_INFO_STREAM("Setting auto-exposure to _on_ / Once");
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAuto", "Once");
    } else {
      NODELET_INFO_STREAM("Setting auto-exposure to _on_ / Continuous");
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAuto",
                                             "Continuous");
    }

    if (exposure_ms > 0) {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAutoLimitAuto",
                                             "Off");
      GenApi::CFloatPtr pExposureUpperLimit =
          pDevice_->GetNodeMap()->GetNode("ExposureAutoUpperLimit");
      if (GenApi::IsWritable(pExposureUpperLimit)) {
        // The parameter in the camera is in us
        pExposureUpperLimit->SetValue(static_cast<int64_t>(exposure_ms) * 1000);
      } else {
        NODELET_INFO("ExposureAutoUpperLimit is not writeable");
      }

    } else {
      // Use automatic auto-exposure limits
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAutoLimitAuto",
                                             "Continuous");
    }

    NODELET_INFO_STREAM(
        "Enabling autoexposure with limits "
        << Arena::GetNodeValue<double>(pNodeMap, "ExposureAutoLowerLimit")
        << " to "
        << Arena::GetNodeValue<double>(pNodeMap, "ExposureAutoUpperLimit"));
  }
}

//===================================================================
//
// Get/set gain

float ArenaCameraNodeletBase::currentGain() {
  GenApi::CFloatPtr pGain = pDevice_->GetNodeMap()->GetNode("Gain");

  if (!pGain || !GenApi::IsReadable(pGain)) {
    NODELET_WARN_STREAM("No gain value");
    return -1.;
  } else {
    float gainValue = pGain->GetValue();
    return gainValue;
  }
}


bool ArenaCameraNodeletBase::setGain(
    ArenaCameraNodeletBase::AutoGainMode gain_mode, float target_gain) {
  try {
    auto pNodeMap = pDevice_->GetNodeMap();

    Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "GainAuto",
                                           "Off");

    if (gain_mode == ArenaCameraNodeletBase::AutoGainMode::Off) {
      NODELET_INFO_STREAM("Setting auto-gain to _off_");
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "GainAuto", "Off");
      // todo update parameter on the server

      GenApi::CFloatPtr pGain = pDevice_->GetNodeMap()->GetNode("Gain");
      float truncated_gain = target_gain;

      if (truncated_gain < 0.0) {
        NODELET_WARN_STREAM("Desired gain ("
                            << target_gain
                            << ") out of range [0.0,1.0]! Setting to 0.0");
        target_gain = 0.0;
      } else if (truncated_gain > 1.0) {
        NODELET_WARN_STREAM("Desired gain ("
                            << target_gain
                            << ") out of range [0.0,1.0]! Setting to 1.0");
        target_gain = 1.0;
      }

      const float gain_min = pGain->GetMin(), gain_max = pGain->GetMax();
      float gain_to_set = gain_min + target_gain * (gain_max - gain_min);
      pGain->SetValue(gain_to_set);

    } else if (gain_mode == ArenaCameraNodeletBase::AutoGainMode::Once) {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "GainAuto", "Once");
      NODELET_INFO_STREAM("Setting auto-gain to _on_ / Once");

    } else if (gain_mode == ArenaCameraNodeletBase::AutoGainMode::Continuous) {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "GainAuto",
                                             "Continuous");
      NODELET_INFO_STREAM("Setting auto-gain to _on_ / Continuous");
    } else {
    }

  } catch (const GenICam::GenericException &e) {
    NODELET_ERROR_STREAM(
        "An exception while setting gain: " << e.GetDescription());
    return false;
  }
  return true;
}

//===================================================================
//
// Get/set Gamma

float ArenaCameraNodeletBase::currentGamma() {
  GenApi::CFloatPtr pGamma = pDevice_->GetNodeMap()->GetNode("Gamma");

  if (!pGamma || !GenApi::IsReadable(pGamma)) {
    NODELET_WARN_STREAM("No gamma value, returning -1");
    return -1.;
  } else {
    float gammaValue = pGamma->GetValue();
    return gammaValue;
  }
}


bool ArenaCameraNodeletBase::setGamma(const float &target_gamma) {
  // for GigE cameras you have to enable gamma first

  GenApi::CBooleanPtr pGammaEnable =
      pDevice_->GetNodeMap()->GetNode("GammaEnable");
  if (GenApi::IsWritable(pGammaEnable)) {
    pGammaEnable->SetValue(true);
  }

  GenApi::CFloatPtr pGamma = pDevice_->GetNodeMap()->GetNode("Gamma");
  if (!pGamma || !GenApi::IsWritable(pGamma)) {
    NODELET_WARN("Cannot set gamma, it is not writeable");
    return false;
  } else {
    try {
      float gamma_to_set = target_gamma;
      if (pGamma->GetMin() > gamma_to_set) {
        gamma_to_set = pGamma->GetMin();
        NODELET_WARN_STREAM(
            "Desired gamma unreachable! Setting to lower limit: "
            << gamma_to_set);
      } else if (pGamma->GetMax() < gamma_to_set) {
        gamma_to_set = pGamma->GetMax();
        NODELET_WARN_STREAM(
            "Desired gamma unreachable! Setting to upper limit: "
            << gamma_to_set);
      }

      NODELET_INFO_STREAM("Setting gamma to " << gamma_to_set);
      pGamma->SetValue(gamma_to_set);

    } catch (const GenICam::GenericException &e) {
      NODELET_ERROR_STREAM("An exception while setting target gamma to "
                           << target_gamma
                           << " occurred: " << e.GetDescription());
      return false;
    }
  }
  return true;
}


//===================================================================
//
// Get/set ROI

sensor_msgs::RegionOfInterest ArenaCameraNodeletBase::currentROI() {
  sensor_msgs::RegionOfInterest roi;
  // \todo{amarburg}  Broke this by getting rid of a member pImage_
  //                  Need to save as state?
  // roi.width = pImage_->GetWidth();
  // roi.height = pImage_->GetHeight();
  // ;
  // roi.x_offset = pImage_->GetOffsetX();
  // roi.y_offset = pImage_->GetOffsetY();
  return roi;
}

bool ArenaCameraNodeletBase::setROI(
    const sensor_msgs::RegionOfInterest target_roi,
    sensor_msgs::RegionOfInterest &reached_roi) {
  boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);
  // TODO: set ROI
  return true;
}


//===================================================================
//
// Get/set binning

int64_t ArenaCameraNodeletBase::currentBinningX() {
  GenApi::CIntegerPtr BinningHorizontal =
      pDevice_->GetNodeMap()->GetNode("BinningHorizontal");

  if (!BinningHorizontal || !GenApi::IsReadable(BinningHorizontal)) {
    NODELET_WARN_STREAM("No binningY value, returning -1");
    return -1;
  } else {
    float binningXValue = BinningHorizontal->GetValue();
    return binningXValue;
  }
}

int64_t ArenaCameraNodeletBase::currentBinningY() {
  GenApi::CIntegerPtr BinningVertical =
      pDevice_->GetNodeMap()->GetNode("BinningVertical");

  if (!BinningVertical || !GenApi::IsReadable(BinningVertical)) {
    NODELET_WARN_STREAM("No binningY value, returning -1");
    return -1;
  } else {
    float binningYValue = BinningVertical->GetValue();
    return binningYValue;
  }
}

bool ArenaCameraNodeletBase::setBinningXValue(const size_t &target_binning_x,
                                              size_t &reached_binning_x) {
  try {
    GenApi::CIntegerPtr pBinningHorizontal =
        pDevice_->GetNodeMap()->GetNode("BinningHorizontal");
    if (GenApi::IsWritable(pBinningHorizontal)) {
      size_t binning_x_to_set = target_binning_x;
      if (binning_x_to_set < pBinningHorizontal->GetMin()) {
        NODELET_WARN_STREAM("Desired horizontal binning_x factor("
                            << binning_x_to_set
                            << ") unreachable! Setting to lower "
                            << "limit: " << pBinningHorizontal->GetMin());
        binning_x_to_set = pBinningHorizontal->GetMin();
      } else if (binning_x_to_set > pBinningHorizontal->GetMax()) {
        NODELET_WARN_STREAM("Desired horizontal binning_x factor("
                            << binning_x_to_set
                            << ") unreachable! Setting to upper "
                            << "limit: " << pBinningHorizontal->GetMax());
        binning_x_to_set = pBinningHorizontal->GetMax();
      }

      pBinningHorizontal->SetValue(binning_x_to_set);
      reached_binning_x = currentBinningX();
    } else {
      NODELET_WARN_STREAM("Camera does not support binning. Will keep the "
                          << "current settings");
      reached_binning_x = currentBinningX();
    }
  }

  catch (const GenICam::GenericException &e) {
    NODELET_ERROR_STREAM("An exception while setting target horizontal "
                         << "binning_x factor to " << target_binning_x
                         << " occurred: " << e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNodeletBase::setBinningX(const size_t &target_binning_x,
                                         size_t &reached_binning_x) {
  boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);

  if (!setBinningXValue(target_binning_x, reached_binning_x)) {
    // retry till timeout
    ros::Rate r(10.0);
    ros::Time timeout(ros::Time::now() + ros::Duration(2.0));
    while (ros::ok()) {
      if (setBinningXValue(target_binning_x, reached_binning_x)) {
        break;
      }
      if (ros::Time::now() > timeout) {
        NODELET_ERROR_STREAM("Error in setBinningX(): Unable to set target "
                             << "binning_x factor before timeout");
        CameraInfoPtr cam_info(
            new CameraInfo(camera_info_manager_->getCameraInfo()));
        cam_info->binning_x = currentBinningX();
        camera_info_manager_->setCameraInfo(*cam_info);
        //   img_raw_msg_.width = pImage_->GetWidth();
        //  step = full row length in bytes, img_size = (step * rows),
        //  imagePixelDepth already contains the number of channels
        //  img_raw_msg_.step = img_raw_msg_.width *
        //  (pImage_->GetBitsPerPixel() / 8);
        return false;
      }
      r.sleep();
    }
  }

  return true;
}

bool ArenaCameraNodeletBase::setBinningYValue(const size_t &target_binning_y,
                                              size_t &reached_binning_y) {
  try {
    GenApi::CIntegerPtr pBinningVertical =
        pDevice_->GetNodeMap()->GetNode("BinningVertical");
    if (GenApi::IsWritable(pBinningVertical)) {
      size_t binning_y_to_set = target_binning_y;
      if (binning_y_to_set < pBinningVertical->GetMin()) {
        NODELET_WARN_STREAM("Desired horizontal binning_y factor("
                            << binning_y_to_set
                            << ") unreachable! Setting to lower "
                            << "limit: " << pBinningVertical->GetMin());
        binning_y_to_set = pBinningVertical->GetMin();
      } else if (binning_y_to_set > pBinningVertical->GetMax()) {
        NODELET_WARN_STREAM("Desired horizontal binning_y factor("
                            << binning_y_to_set
                            << ") unreachable! Setting to upper "
                            << "limit: " << pBinningVertical->GetMax());
        binning_y_to_set = pBinningVertical->GetMax();
      }

      pBinningVertical->SetValue(binning_y_to_set);
      reached_binning_y = currentBinningY();
    } else {
      NODELET_WARN_STREAM("Camera does not support binning. Will keep the "
                          << "current settings");
      reached_binning_y = currentBinningY();
    }
  }

  catch (const GenICam::GenericException &e) {
    NODELET_ERROR_STREAM("An exception while setting target horizontal "
                         << "binning_y factor to " << target_binning_y
                         << " occurred: " << e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNodeletBase::setBinningY(const size_t &target_binning_y,
                                         size_t &reached_binning_y) {
  boost::lock_guard<boost::recursive_mutex> lock(device_mutex_);

  if (!setBinningYValue(target_binning_y, reached_binning_y)) {
    // retry till timeout
    ros::Rate r(10.0);
    ros::Time timeout(ros::Time::now() + ros::Duration(2.0));
    while (ros::ok()) {
      if (setBinningYValue(target_binning_y, reached_binning_y)) {
        break;
      }
      if (ros::Time::now() > timeout) {
        NODELET_ERROR_STREAM("Error in setBinningY(): Unable to set target "
                             << "binning_y factor before timeout");
        CameraInfoPtr cam_info(
            new CameraInfo(camera_info_manager_->getCameraInfo()));
        cam_info->binning_y = currentBinningY();
        camera_info_manager_->setCameraInfo(*cam_info);

        // img_raw_msg_.width = pImage_->GetWidth();
        // //  step = full row length in bytes, img_size = (step * rows),
        // //  imagePixelDepth already contains the number of channels
        // img_raw_msg_.step =
        //     img_raw_msg_.width * (pImage_->GetBitsPerPixel() / 8);
        return false;
      }
      r.sleep();
    }
  }

  return true;
}

//~~ Brightness / auto controls ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void ArenaCameraNodeletBase::setTargetBrightness(unsigned int brightness) {
  // const bool was_streaming = is_streaming_;
  // stopStreaming();

  try {
    GenApi::CIntegerPtr pTargetBrightness =
        pDevice_->GetNodeMap()->GetNode("TargetBrightness");

    if (GenApi::IsWritable(pTargetBrightness)) {
      if (brightness > 255) brightness = 255;
      pTargetBrightness->SetValue(brightness);

      NODELET_INFO_STREAM("Set target brightness to "
                          << pTargetBrightness->GetValue());
    } else {
      NODELET_INFO_STREAM("TargetBrightness is not writeable; current value "
                          << pTargetBrightness->GetValue());
    }
  } catch (const GenICam::GenericException &e) {
    NODELET_ERROR_STREAM(
        "An exception while setting TargetBrightness: " << e.GetDescription());
  }

  // if (was_streaming)
  //   startStreaming();
}


void ArenaCameraNodeletBase::disableAllRunningAutoBrightessFunctions() {
  GenApi::CStringPtr pExposureAuto = pNodeMap_->GetNode("ExposureAuto");
  GenApi::CStringPtr pGainAuto = pNodeMap_->GetNode("GainAuto");

  if (!pExposureAuto || !GenApi::IsWritable(pExposureAuto) || !pGainAuto ||
      !GenApi::IsWritable(pGainAuto)) {
    NODELET_WARN_STREAM("Unable to disable auto gain & exposure");
    return;
  }

  else {
    Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(),
                                           "ExposureAuto", "Off");
    Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "GainAuto",
                                           "Off");
  }
}


//-------------------------------------------------------------------
//
// HDR Channel Query and set functions
//
float ArenaCameraNodeletBase::currentHdrGain(int channel) {
  try {
    Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(),
                                 "HDRTuningChannelSelector", channel);

    return Arena::GetNodeValue<double>(pDevice_->GetNodeMap(),
                                       "HDRChannelAnalogGain");
  } catch (const GenICam::GenericException &e) {
    NODELET_ERROR_STREAM(
        "Exception while querying HDR gain: " << e.GetDescription());
    return -1;
  }
}

float ArenaCameraNodeletBase::currentHdrExposure(int channel) {
  try {
    Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(),
                                 "HDRTuningChannelSelector", channel);

    return Arena::GetNodeValue<double>(pDevice_->GetNodeMap(),
                                       "HDRChannelExposureTime");
  } catch (const GenICam::GenericException &e) {
    NODELET_ERROR_STREAM(
        "Exception while querying HDR exposure time: " << e.GetDescription());
    return -1;
  }
}

//-------------------------------------------------------------------
// Functions for dealing with LUT
//
// \todo{amarburg}  Very simple right now
//

void ArenaCameraNodeletBase::enableLUT(bool enable) {
  try {
    Arena::SetNodeValue<bool>(pDevice_->GetNodeMap(), "LUTEnable", enable);
  } catch (const GenICam::GenericException &e) {
    NODELET_ERROR_STREAM("An exception while setting LUTEnable to "
                         << (enable ? "true" : "false")
                         << " occurred: " << e.GetDescription());
  }
}

//------------------------------------------------------------------------
//  ROS Reconfigure callback
//

void ArenaCameraNodeletBase::reconfigureCallback(ArenaCameraConfig &config,
                                                 uint32_t level) {
  const auto stop_level =
      (uint32_t)dynamic_reconfigure::SensorLevels::RECONFIGURE_STOP;

  const bool was_streaming = is_streaming_;
  if (level >= stop_level) {
    ROS_INFO("Stopping sensor for reconfigure");
    stopStreaming();
  }

  NODELET_INFO_STREAM("In reconfigureCallback");

  // -- The following params require stopping streaming, only set if needed --
  if (config.frame_rate != previous_config_.frame_rate) {
    arena_camera_parameter_set_.setFrameRate(config.frame_rate);
    updateFrameRate();
  }

  setTargetBrightness(config.target_brightness);

  arena_camera_parameter_set_.exposure_auto_ = config.auto_exposure;
  arena_camera_parameter_set_.exposure_ms_ = config.exposure_ms;
  arena_camera_parameter_set_.auto_exposure_max_ms_ =
      config.auto_exposure_max_ms;

  if (config.auto_exposure) {
    setExposure(ArenaCameraNodeletBase::AutoExposureMode::Continuous,
                config.auto_exposure_max_ms);
  } else {
    setExposure(ArenaCameraNodeletBase::AutoExposureMode::Off,
                config.exposure_ms);
  }
  
  arena_camera_parameter_set_.gain_auto_ = config.auto_gain;
  if (arena_camera_parameter_set_.gain_auto_) {
    setGain(ArenaCameraNodeletBase::AutoGainMode::Continuous);
  } else {
    setGain(ArenaCameraNodeletBase::AutoGainMode::Off, config.gain);
  }

  arena_camera_parameter_set_.gamma_ = config.gamma;
  setGamma(arena_camera_parameter_set_.gamma_);

  if ((level >= stop_level) && was_streaming) {
    startStreaming();
  }

  // Save config
  previous_config_ = config;
}

//------------------------------------------------------------------------
//  ROS Disagnostics callbacks
//

void ArenaCameraNodeletBase::create_diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {}

void ArenaCameraNodeletBase::create_camera_info_diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if (camera_info_manager_->isCalibrated()) {
    stat.summaryf(DiagnosticStatus::OK, "Intrinsic calibration found");
  } else {
    stat.summaryf(DiagnosticStatus::ERROR, "No intrinsic calibration found");
  }
}

void ArenaCameraNodeletBase::diagnostics_timer_callback_(
    const ros::TimerEvent &) {
  diagnostics_updater_.update();
}

}  // namespace arena_camera
