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

#pragma once
#include "ca2lib/types.h"
#include <sensor_msgs/PointCloud2.h>

namespace ca2lib {

/**
 * @brief Converts a sensor_msg::PointCloud2 into PointCloudXf format. Each
 * field is casted to float but original datatypes are kept in case cloud has to
 * be republished.
 *
 * @param msg_ const pointer to a sensor_msgs::PointCloud2
 * @return PointCloudXf
 */
PointCloudXf convertRosToPointCloud(
    const sensor_msgs::PointCloud2::ConstPtr& msg_);

/**
 * @brief Converts a PointCloudXf to a sensor_msgs::PointCloud2 type for ROS
 * publishing.
 *
 * @param cloud_
 * @param msg_
 */
void convertPointCloudXfToRos(const PointCloudXf& cloud_,
                              sensor_msgs::PointCloud2& msg_);
}  // namespace ca2lib