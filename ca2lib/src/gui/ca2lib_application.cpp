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

#include "ca2lib/gui/ca2lib_application.h"
#include <QHBoxLayout>
#include <cv_bridge/cv_bridge.h>
#include "ca2lib/projection.h"
#include <opencv2/highgui.hpp>
namespace ca2lib {

Ca2libApplication::Ca2libApplication(const std::string& topic_rgb_,
                                     const std::string topic_cloud_,
                                     QWidget* parent)
    : QWidget(parent) {
  ros::NodeHandle nh;
  _sub_rgb = nh.subscribe<sensor_msgs::Image>(
      topic_rgb_, 10, &Ca2libApplication::onNewImage, this);
  _sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>(
      topic_cloud_, 10, &Ca2libApplication::onNewCloud, this);

  _view_rgb.setMinimumSize(80, 60);
  _view_cloud.setMinimumSize(80, 60);

  _view_cloud.enableSelection(true);
  _widget = new QWidget(this);
  QHBoxLayout* hlay = new QHBoxLayout(_widget);
  hlay->setStretch(0, 1);
  hlay->setStretch(1, 1);
  hlay->setStretch(2, 1);

  hlay->addWidget(&_view_rgb);
  hlay->addWidget(&_view_cloud);
  hlay->addWidget(&_view_reprojection);

  _view_rgb.setOuterLayout(hlay);
  _view_cloud.setOuterLayout(hlay);
  _view_reprojection.setOuterLayout(hlay);

  setMinimumSize(hlay->sizeHint());
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void Ca2libApplication::onNewImage(const sensor_msgs::Image::ConstPtr& msg_) {
  cv_bridge::CvImageConstPtr img_ptr = cv_bridge::toCvShare(msg_, "rgb8");

  const cv::Mat& im = img_ptr->image;
  QImage image = QImage((uchar*)im.data, im.cols, im.rows, im.step,
                        QImage::Format::Format_RGB888);
  std::cerr << "Received new image" << std::endl;
  //   _view_rgb.setImage(image);
}

void Ca2libApplication::onNewCloud(
    const sensor_msgs::PointCloud2::ConstPtr& msg_) {
  _cloud = convertRosToPointCloud(msg_);
  _lut = projectLidarLUT(_cloud, _ilut);
  cv::Mat range =
      composeChannelImage(_cloud, _lut, "range", ca2lib::MINMAX, 0, 10000);
  range.convertTo(_range, CV_8UC3, 255, 0);

  std::cerr << "Received new cloud" << std::endl;

  QImage image = QImage((uchar*)_range.data, _range.cols, _range.rows,
                        _range.step, QImage::Format::Format_RGB888);
  _view_cloud.setImage(image);
}
}  // namespace ca2lib