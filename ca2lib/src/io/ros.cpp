// clang-format off

// Copyright (c) 2023, S(apienza) R(obust) R(obotics) G(roup)

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

#include "ca2lib/io/ros.h"
#include <functional>

namespace ca2lib {

std::function<float(const uint8_t*)> _field_conversion[] = {
    nullptr,
    [](const uint8_t* a) { return (float)(*(int8_t*)a); },
    [](const uint8_t* a) { return (float)(*(uint8_t*)a); },
    [](const uint8_t* a) { return (float)(*(int16_t*)a); },
    [](const uint8_t* a) { return (float)(*(uint16_t*)a); },
    [](const uint8_t* a) { return (float)(*(int32_t*)a); },
    [](const uint8_t* a) { return (float)(*(uint32_t*)a); },
    [](const uint8_t* a) { return (float)(*(float*)a); },
    [](const uint8_t* a) { return (float)(*(double*)a); }};

unsigned long _field_size[] = {0,
                               sizeof(int8_t),
                               sizeof(uint8_t),
                               sizeof(int16_t),
                               sizeof(uint16_t),
                               sizeof(int32_t),
                               sizeof(uint32_t),
                               sizeof(float),
                               sizeof(double)};

std::function<void(const float, uint8_t*)> _field_float2val[] = {
    nullptr,
    [](const float v, uint8_t* buf) { *(int8_t*)buf = (int8_t)(v); },
    [](const float v, uint8_t* buf) { *(uint8_t*)buf = (uint8_t)(v); },
    [](const float v, uint8_t* buf) { *(int16_t*)buf = (int16_t)(v); },
    [](const float v, uint8_t* buf) { *(uint16_t*)buf = (uint16_t)(v); },
    [](const float v, uint8_t* buf) { *(int32_t*)buf = (int32_t)(v); },
    [](const float v, uint8_t* buf) { *(uint32_t*)buf = (uint32_t)(v); },
    [](const float v, uint8_t* buf) { *(float*)buf = (float)(v); },
    [](const float v, uint8_t* buf) { *(double*)buf = (double)(v); }};

/**
 * @brief Converts a sensor_msg::PointCloud2 into PointCloudXf format. Each
 * field is casted to float but original datatypes are kept in case cloud has to
 * be republished.
 *
 * @param msg_ const pointer to a sensor_msgs::PointCloud2
 * @return PointCloudXf
 */
PointCloudXf convertRosToPointCloud(
    const sensor_msgs::PointCloud2::ConstPtr& msg_) {
  unsigned int num_fields = msg_->fields.size();
  unsigned int num_points = msg_->height * msg_->width;

  PointCloudXf cloud;
  cloud.points.resize(num_points);
  cloud.height = msg_->height;
  cloud.width = msg_->width;

  for (unsigned int i = 0; i < msg_->fields.size(); ++i) {
    auto field_name = msg_->fields[i].name;
    auto field_type = msg_->fields[i].datatype;
    cloud.fields[field_name] = std::make_pair(i, field_type);
  }

  for (unsigned int i = 0; i < num_points; ++i) {
    const uint8_t* base_ptr = msg_->data.data() + i * msg_->point_step;
    Eigen::VectorXf p(num_fields);
    for (size_t j = 0; j < num_fields; ++j) {
      const uint8_t* src = base_ptr + msg_->fields[j].offset;
      p(j) = _field_conversion[msg_->fields[j].datatype](src);
    }
    cloud.points[i] = p;
  }
  return cloud;
}

/**
 * @brief Converts a PointCloudXf to a sensor_msgs::PointCloud2 type for ROS
 * publishing.
 *
 * @param cloud_
 * @param msg_
 */
void convertPointCloudXfToRos(const PointCloudXf& cloud_,
                              sensor_msgs::PointCloud2& msg_) {
  msg_.fields.resize(cloud_.fields.size());
  // Convert map to an ordered vector [KEY, Index, Type]
  std::vector<std::tuple<std::string, unsigned int, uint8_t>> map_rasterized(
      cloud_.fields.size());
  for (auto it = cloud_.fields.begin(); it != cloud_.fields.end(); ++it) {
    map_rasterized[it->second.first] =
        std::make_tuple(it->first, it->second.first, it->second.second);
  }
  // Build field table
  unsigned int offset = 0;
  for (unsigned int i = 0; i < cloud_.fields.size(); ++i) {
    auto& field = msg_.fields[i];
    field.name = std::get<0>(map_rasterized[i]);
    field.offset = offset;
    field.datatype = std::get<2>(map_rasterized[i]);
    field.count = 1;
    offset += _field_size[field.datatype];
  }
  msg_.height = cloud_.height;
  msg_.width = cloud_.width;
  msg_.is_bigendian = false;
  msg_.is_dense = false;
  msg_.point_step = offset;
  msg_.row_step = msg_.point_step * cloud_.width;
  msg_.data.resize(msg_.row_step * msg_.height);
  // Insert points
  for (unsigned int i = 0; i < cloud_.points.size(); ++i) {
    const auto& p = cloud_.points[i];
    uint8_t* base_ptr = msg_.data.data() + i * msg_.point_step;
    for (unsigned j = 0; j < msg_.fields.size(); ++j) {
      auto base_offset = msg_.fields[j].offset;
      _field_float2val[msg_.fields[j].datatype](p(j), base_ptr + base_offset);
    }
  }
}
}  // namespace ca2lib