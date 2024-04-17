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

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <opencv2/highgui.hpp>

#include "ca2lib/parse_command_line.h"
#include "ca2lib/targets.h"

using namespace srrg2_core;

const char* whatdoes =
    "Record images from a single camera. Used to store images for "
    "calibration";

void image_callback(const sensor_msgs::ImageConstPtr& msg);

std::filesystem::path output_dir;
cv::Mat detection_mask;
ca2lib::TargetBase::SharedPtr target_ptr = nullptr;

int main(int argc, char** argv) {
  ros::init(argc, argv, "mono_recorder");
  ros::NodeHandle nh("/");

  ParseCommandLine parser(argv, &whatdoes);

  ArgumentString topic_cam(&parser, "c", "topic-camera",
                           "sensor_msgs/Image topic", "");
  ArgumentString target_f(&parser, "t", "target",
                          "YAML file containing target metadata", "");
  ArgumentString odir(&parser, "o", "output-dir", "Output destination folder",
                      ".");

  parser.parse();

  if (!odir.isSet()) {
    std::cerr << "No output destination set.\n"
              << parser.options() << std::endl;
    return -1;
  }

  spdlog::info("Creating output directory " + odir.value());

  output_dir = odir.value();
  std::filesystem::create_directories(output_dir);

  if (!target_f.isSet()) {
    spdlog::warn("No target provided, target detection will not be available.");
  } else {
    target_ptr = ca2lib::TargetBase::fromFile(target_f.value());
  }

  ros::Subscriber sub_img =
      nh.subscribe<sensor_msgs::Image>(topic_cam.value(), 10, image_callback);

  ros::spin();

  return 0;
}

void image_callback(const sensor_msgs::ImageConstPtr& msg) {
  const cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

  cv::Mat viz_frame = frame.clone();

  const auto& detections = target_ptr->getData();

  detections.drawDetection(viz_frame);

  cv::resize(viz_frame, viz_frame,
             cv::Size(viz_frame.cols / 2, viz_frame.rows / 2));

  cv::imshow("Mono", viz_frame);

  char key = cv::waitKey(10);

  const auto ts = msg->header.stamp.toSec();

  switch (key) {
    case 0x20:  // Space
      if (target_ptr->detect(frame)) {
        target_ptr->saveDetection();
        std::filesystem::path dest = output_dir / (std::to_string(ts) + ".png");
        spdlog::info("Saving " + dest.string());
        cv::imwrite(dest.string(), frame);
      } else {
        spdlog::warn("Could not detect target on frame");
      }
      break;
    case 0x1B:  // Esc
      spdlog::info("Closing the program");
      ros::shutdown();
      break;

    case 'h':
      std::cout
          << "press space for detection, esc to close, 'h' to print this line"
          << std::endl;
      break;
  }
}
