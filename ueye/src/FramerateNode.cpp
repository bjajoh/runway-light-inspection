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

#include <ueye/FramerateNode.h>

namespace ueye
{

FramerateNode::FramerateNode(ros::NodeHandle node, ros::NodeHandle priv_nh) :
    first_(true), rate_(0.0), stamp_old_(0)
{
  // Grab the topic name from the ROS parameter
  std::string topic = std::string("image_raw");
  priv_nh.getParam("topic", topic);

  // Set up Subscribers
  sub_ = node.subscribe(topic, 2, &FramerateNode::imageRecv, this, ros::TransportHints().tcpNoDelay(true));
}

FramerateNode::~FramerateNode()
{
}

void FramerateNode::imageRecv(const sensor_msgs::Image::ConstPtr& msg)
{
  ros::Time stamp = ros::Time::now();
  if (first_) {
    first_ = false;
  } else {
    double temp_rate = (double)1e9 / (double)(stamp - stamp_old_).toNSec();
    if (rate_ == 0.0) {
      rate_ = temp_rate;
    } else {
      rate_ += (temp_rate - rate_) * 0.2;
    }
  }
  stamp_old_ = stamp;

  ROS_INFO("%d %dx%d at %0.2fHz", msg->header.seq, msg->width, msg->height, rate_);
}

} // namespace ueye
