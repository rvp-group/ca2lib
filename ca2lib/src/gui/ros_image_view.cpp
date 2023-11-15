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

#include "ca2lib/gui/ros_image_view.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

namespace ca2lib {

void RosImageView::subscribe(const std::string& topic_) {
  ros::NodeHandle nh;
  std::cerr << "Subscribing to " << topic_ << std::endl;
  _sub_image = nh.subscribe<sensor_msgs::Image>(
      topic_, 10, &RosImageView::onRosImage, this);
}

void RosImageView::onRosImage(const sensor_msgs::Image::ConstPtr& msg_) {
  cv_bridge::CvImageConstPtr img_ptr = cv_bridge::toCvShare(msg_, "rgb8");

  const cv::Mat& im = img_ptr->image;
  QImage image = QImage((uchar*)im.data, im.cols, im.rows, im.step,
                        QImage::Format::Format_RGB888);
  std::cerr << "Received new image" << std::endl;
  _frame->setImage(image);
}

RosImageView::RosImageView(QWidget* parent, Qt::WindowFlags flags)
    : QWidget(parent, flags) {
  _window = new QWidget(this);
  _window->setGeometry(0, 0, 425, 300);
  QVBoxLayout* vlay = new QVBoxLayout(_window);
  // QPushButton* button0 = new QPushButton("Frocio");
  _frame = new ImageFrame;
  // vlay->addWidget(button0);
  vlay->addWidget(_frame);
}
}  // namespace ca2lib