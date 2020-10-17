/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-2016, Kevin Hallenbeck
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Kevin Hallenbeck nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ueye/CameraNode.h>
#include <boost/format.hpp>

namespace ueye
{

CameraNode::CameraNode(ros::NodeHandle node, ros::NodeHandle priv_nh) :
    srv_(priv_nh), it_(node)
{
  running_ = false;
  configured_ = false;
  force_streaming_ = false;
  auto_exposure_ = false;
  auto_gain_ = false;
  trigger_mode_ = zoom_ = -1;

  // Check for a valid uEye installation and supported version
  const char *version;
  int major, minor, build;
  if (cam_.checkVersion(major, minor, build, version)) {
    ROS_INFO("Loaded uEye SDK %s.", version);
  } else {
    ROS_WARN("Loaded uEye SDK %d.%d.%d. Expecting %s.", major, minor, build, version);
  }

  // Make sure there is at least one camera available
  int num_cameras = cam_.getNumberOfCameras();
  if (num_cameras > 0) {
    if (num_cameras == 1) {
      ROS_INFO("Found 1 uEye camera.");
    } else {
      ROS_INFO("Found %d uEye cameras.", num_cameras);
    }
  } else {
    ROS_ERROR("Found 0 uEye cameras.");
    ros::shutdown();
    return;
  }

  // Open camera with either serialNo, deviceId, or cameraId
  int id = 0;
  if (priv_nh.getParam("serialNo", id)) {
    if (!cam_.openCameraSerNo(id)) {
      ROS_ERROR("Failed to open uEye camera with serialNo: %u.", id);
      ros::shutdown();
      return;
    }
  } else if (priv_nh.getParam("deviceId", id)) {
    if (!cam_.openCameraDevId(id)) {
      ROS_ERROR("Failed to open uEye camera with deviceId: %u.", id);
      ros::shutdown();
      return;
    }
  } else {
    priv_nh.getParam("cameraId", id);
    if (!cam_.openCameraCamId(id)) {
      ROS_ERROR("Failed to open uEye camera with cameraId: %u.", id);
      ros::shutdown();
      return;
    }
  }
  ROS_INFO("Opened camera %s %u", cam_.getCameraName(), cam_.getCameraSerialNo());

  // Setup Dynamic Reconfigure
  dynamic_reconfigure::Server<monoConfig>::CallbackType f = boost::bind(&CameraNode::reconfig, this, _1, _2);
  srv_.setCallback(f);	//start dynamic reconfigure

  // Service call for setting calibration.
  srv_cam_info_ = node.advertiseService("set_camera_info", &CameraNode::setCameraInfo, this);

  // Special publisher for images to support compression
  pub_stream_ = it_.advertiseCamera("image_raw", 0);

  // Set up Timer
  timer_ = node.createTimer(ros::Duration(1 / 5.0), &CameraNode::timerCallback, this);
}

CameraNode::~CameraNode()
{
  closeCamera();
}

void CameraNode::handlePath(std::string &path)
{
  // Set default path if not present
  if (path.length() == 0) {
    path = ros::package::getPath("ueye");
  }

  // Remove trailing "/" from folder path
  unsigned int length = path.length();
  if (length > 0) {
    if (path.c_str()[length - 1] == '/') {
      path = path.substr(0, length - 1);
    }
  }
  config_path_ = path;
}
void CameraNode::reconfig(monoConfig &config, uint32_t level)
{
  force_streaming_ = config.force_streaming;
  handlePath(config.config_path);

  // Trigger
  if (trigger_mode_ != config.trigger) {
    stopCamera();
    TriggerMode mode;
    switch (config.trigger) {
      case mono_TGR_HARDWARE_RISING:
        mode = TRIGGER_LO_HI;
        break;
      case mono_TGR_HARDWARE_FALLING:
        mode = TRIGGER_HI_LO;
        break;
      case mono_TGR_AUTO:
      default:
        mode = TRIGGER_OFF;
        break;
    }
    if (!cam_.setTriggerMode(mode)) {
      cam_.setTriggerMode(TRIGGER_OFF);
      config.trigger = mono_TGR_AUTO;
    }
  }
  trigger_mode_ = config.trigger;

  // Color Mode
  uEyeColor color;
  switch (config.color) {
    default:
    case mono_COLOR_MONO8:
      color = MONO8;
      break;
    case mono_COLOR_MONO16:
      color = MONO16;
      break;
    case mono_COLOR_YUV:
      color = YUV;
      break;
    case mono_COLOR_YCbCr:
      color = YCbCr;
      break;
    case mono_COLOR_BGR5:
      color = BGR5;
      break;
    case mono_COLOR_BGR565:
      color = BGR565;
      break;
    case mono_COLOR_BGR8:
      color = BGR8;
      break;
    case mono_COLOR_BGRA8:
      color = BGRA8;
      break;
    case mono_COLOR_BGRY8:
      color = BGRY8;
      break;
    case mono_COLOR_RGB8:
      color = RGB8;
      break;
    case mono_COLOR_RGBA8:
      color = RGBA8;
      break;
    case mono_COLOR_RGBY8:
      color = RGBY8;
      break;
  }
  if (cam_.getColorMode() != color) {
    cam_.setColorMode(color);
  }

  // Latch Auto Parameters
  if (auto_gain_ && !config.auto_gain) {
    config.gain = cam_.getHardwareGain();
  }
  auto_gain_ = config.auto_gain;
  if (auto_exposure_ && !config.auto_exposure) {
    config.exposure_time = cam_.getExposure();
  }
  auto_exposure_ = config.auto_exposure;

  // Hardware Gamma Correction
  if (cam_.getHardwareGamma() != config.hardware_gamma) {
    cam_.setHardwareGamma(&config.hardware_gamma);
  }

  // Gain Boost
  if (cam_.getGainBoost() != config.gain_boost) {
    cam_.setGainBoost(&config.gain_boost);
  }

  // Hardware Gain
  if (cam_.getAutoGain() != config.auto_gain) {
    cam_.setAutoGain(&config.auto_gain);
  }
  if (!config.auto_gain) {
    cam_.setHardwareGain(&config.gain);
  }

  // Zoom
  if (cam_.getZoom() != config.zoom) {
    cam_.setZoom(&config.zoom);
  }

  // Pixel Clock
  if (cam_.getPixelClock() != config.pixel_clock) {
    cam_.setPixelClock(&config.pixel_clock);
  }

  // Frame Rate
  cam_.setFrameRate(&config.frame_rate);

  // Exposure
  if (cam_.getAutoExposure() != config.auto_exposure) {
    cam_.setAutoExposure(&config.auto_exposure);
  }
  if (!config.auto_exposure) {
    cam_.setExposure(&config.exposure_time);
  }

  // Zoom
  if (zoom_ != config.zoom) {
    zoom_ = config.zoom;
    loadIntrinsics();
  }

  msg_camera_info_.header.frame_id = config.frame_id;
  configured_ = true;
}

void CameraNode::timerCallback(const ros::TimerEvent& event)
{
  if ((pub_stream_.getNumSubscribers() > 0) || force_streaming_) {
    startCamera();
  } else {
    stopCamera();
  }
}

// Support camera calibration requests
// http://www.ros.org/wiki/camera_calibration/Tutorials/MonocularCalibration
bool CameraNode::setCameraInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp)
{
  ROS_INFO("New camera info received");
  sensor_msgs::CameraInfo &info = req.camera_info;
  info.header.frame_id = msg_camera_info_.header.frame_id;

  // Sanity check: the image dimensions should match the resolution of the sensor.
  unsigned int height = cam_.getHeight();
  unsigned int width = cam_.getWidth();

  if (info.width != width || info.height != height) {
    rsp.success = false;
    rsp.status_message = (boost::format("Camera_info resolution %ix%i does not match current video "
                                        "setting, camera running at resolution %ix%i.") % info.width % info.height
        % width % height).str();
    ROS_ERROR("%s", rsp.status_message.c_str());
    return true;
  }

  std::string camname = cam_.getCameraName();
  std::stringstream ini_stream;
  if (!camera_calibration_parsers::writeCalibrationIni(ini_stream, camname, info)) {
    rsp.status_message = "Error formatting camera_info for storage.";
    rsp.success = false;
  } else {
    std::string ini = ini_stream.str();
    std::fstream param_file;
    std::string filename = config_path_ + "/" + configFileName(cam_);
    param_file.open(filename.c_str(), std::ios::in | std::ios::out | std::ios::trunc);

    if (param_file.is_open()) {
      param_file << ini.c_str();
      param_file.close();

      msg_camera_info_ = info;
      rsp.success = true;
    } else {
      rsp.success = false;
      rsp.status_message = "file write failed";
    }
  }
  if (!rsp.success) {
    ROS_ERROR("%s", rsp.status_message.c_str());
  }
  return true;
}

// Try to load previously saved camera calibration from a file.
void CameraNode::loadIntrinsics()
{
  char buffer[12800];

  std::string MyPath = config_path_ + "/" + configFileName(cam_);
  std::fstream param_file;
  param_file.open(MyPath.c_str(), std::ios::in);

  if (param_file.is_open()) {
    param_file.read(buffer, 12800);
    param_file.close();
  }

  // Parse calibration file
  std::string camera_name;
  if (camera_calibration_parsers::parseCalibrationIni(buffer, camera_name, msg_camera_info_)) {
    ROS_INFO("Loaded calibration for camera '%s'", camera_name.c_str());
  } else {
    ROS_WARN("Failed to load intrinsics for camera from file");
  }
}

// Add properties to image message
sensor_msgs::ImagePtr CameraNode::processFrame(const char *frame, size_t size, sensor_msgs::CameraInfoPtr &info)
{
  msg_camera_info_.header.stamp = ros::Time::now();
  msg_camera_info_.roi.x_offset = 0;
  msg_camera_info_.roi.y_offset = 0;
  msg_camera_info_.height = cam_.getHeight();
  msg_camera_info_.width = cam_.getWidth();
  msg_camera_info_.roi.width = 0;
  msg_camera_info_.roi.height = 0;
  sensor_msgs::CameraInfoPtr msg(new sensor_msgs::CameraInfo(msg_camera_info_));
  info = msg;

  sensor_msgs::ImagePtr msg_image(new sensor_msgs::Image());
  msg_image->header = msg_camera_info_.header;
  msg_image->height = msg_camera_info_.height;
  msg_image->width = msg_camera_info_.width;
  msg_image->encoding = Camera::colorModeToString(cam_.getColorMode());
  msg_image->is_bigendian = false;
  msg_image->step = size / msg_image->height;
  msg_image->data.resize(size);
  memcpy(msg_image->data.data(), frame, size);
  return msg_image;
}

// Timestamp and publish the image. Called by the streaming thread.
void CameraNode::publishImage(const char *frame, size_t size)
{
  sensor_msgs::CameraInfoPtr info;
  sensor_msgs::ImagePtr img = processFrame(frame, size, info);
  pub_stream_.publish(img, info);
}

void CameraNode::startCamera()
{
  if (running_ || !configured_)
    return;
  cam_.startVideoCapture(boost::bind(&CameraNode::publishImage, this, _1, _2));
  ROS_INFO("Started video stream.");
  running_ = true;
}

void CameraNode::stopCamera()
{
  if (!running_)
    return;
  cam_.stopVideoCapture();
  ROS_INFO("Stopped video stream.");
  running_ = false;
}

void CameraNode::closeCamera()
{
  stopCamera();
  cam_.closeCamera();
}

} // namespace ueye
