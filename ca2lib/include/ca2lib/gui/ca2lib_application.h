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

#pragma once
#include "ca2lib/gui/ros_image_view.h"
#include "ca2lib/gui/image_frame.h"
#include <QWidget>
#include <QObject>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include "ca2lib/types.h"
#include "ca2lib/io/ros.h"

namespace ca2lib {
class Ca2libApplication : public QWidget {
  Q_OBJECT

 public:
  Ca2libApplication(const std::string& topic_rgb_,
                    const std::string topic_cloud_, QWidget* parent = nullptr);

 protected:
  void onNewImage(const sensor_msgs::Image::ConstPtr&);
  void onNewCloud(const sensor_msgs::PointCloud2::ConstPtr&);

 private:
  QWidget* _widget;
  ImageFrame _view_rgb, _view_cloud, _view_reprojection;

  ros::Subscriber _sub_rgb, _sub_cloud;

  PointCloudXf _cloud;
  cv::Mat _lut, _range;
  std::vector<std::pair<bool, cv::Point2i>> _ilut;
};
}  // namespace ca2lib