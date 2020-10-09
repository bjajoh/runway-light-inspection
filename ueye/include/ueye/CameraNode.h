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

#ifndef _CAMERA_NODE_H_
#define _CAMERA_NODE_H_

// ROS communication
#include <ros/ros.h>
#include <ros/package.h>	// finds package paths
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse_ini.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <ueye/monoConfig.h>

// File IO
#include <sstream>
#include <fstream>

// ueye::Camera class
#include <ueye/Camera.h>

namespace ueye
{

static std::string configFileName(const Camera &cam) {
  std::stringstream ss;
  ss << "Cal-" << cam.getCameraName() << "-" << cam.getZoom() << "-" << cam.getCameraSerialNo() << ".txt";
  return ss.str();
}

class CameraNode
{
public:
  CameraNode(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~CameraNode();

private:
  // ROS callbacks
  void reconfig(monoConfig &config, uint32_t level);
  void timerCallback(const ros::TimerEvent& event);
  bool setCameraInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp);

  void loadIntrinsics();
  sensor_msgs::ImagePtr processFrame(const char *frame, size_t size, sensor_msgs::CameraInfoPtr &info);
  void publishImage(const char *frame, size_t size);
  void startCamera();
  void stopCamera();
  void closeCamera();
  void handlePath(std::string &path);

  dynamic_reconfigure::Server<monoConfig> srv_;
  ros::Timer timer_;
  sensor_msgs::CameraInfo msg_camera_info_;

  ueye::Camera cam_;
  bool running_;
  bool configured_;
  bool force_streaming_;
  std::string config_path_;
  int trigger_mode_;
  bool auto_exposure_;
  bool auto_gain_;
  int zoom_;

  // ROS topics
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher pub_stream_;
  ros::ServiceServer srv_cam_info_;
};

} // namespace ueye

#endif // _CAMERA_NODE_H_
