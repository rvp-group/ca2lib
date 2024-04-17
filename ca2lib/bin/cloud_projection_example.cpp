// clang-format off

// Copyright (c) 2023, Robotics Vision and Perception Group

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// clang-format on
#include <ros/ros.h>

#include <opencv2/highgui.hpp>

#include "ca2lib/io/ros.h"
#include "ca2lib/projection.h"
#include "ca2lib/types.h"

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg_);

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_projection_example");

  ros::NodeHandle nh;

  ros::Subscriber sub_cloud =
      nh.subscribe<sensor_msgs::PointCloud2>("/cloud_in", 10, cloud_cb);

  ros::spin();
  return 0;
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg_) {
  ca2lib::PointCloudXf cloud = ca2lib::convertRosToPointCloud(msg_);

  const auto lut = ca2lib::projectLidarLUT(cloud);
  const auto ambient_image = ca2lib::composeChannelImage(
      cloud, lut, "ambient", ca2lib::NormalizationType::MINMAX, 0.0f, 200.f);
  const auto intensity_image = ca2lib::composeChannelImage(
      cloud, lut, "intensity", ca2lib::NormalizationType::MINMAX, 0.0f, 200.f);
  const auto range_image = ca2lib::composeChannelImage(
      cloud, lut, "range", ca2lib::NormalizationType::MINMAX, 0.0f, 10000.f);

  cv::imshow("Ambient", ambient_image);
  cv::imshow("Intensity", intensity_image);
  cv::imshow("Range", range_image);
  cv::waitKey(10);
}