#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <opencv2/highgui.hpp>

#include "ca2lib/parse_command_line.h"
#include "ca2lib/targets.h"

using namespace srrg2_core;

const char* whatdoes =
    "Record images from two cameras. Can be used to store images for "
    "calibration";

void sync_callback(const sensor_msgs::ImageConstPtr& left_msg,
                   const sensor_msgs::ImageConstPtr& right_msg);

std::filesystem::path left_output_dir, right_output_dir;
cv::Mat left_detection_mask, right_detection_mask;
ca2lib::TargetBase::SharedPtr left_target_ptr = nullptr;
ca2lib::TargetBase::SharedPtr right_target_ptr = nullptr;

int main(int argc, char** argv) {
  ros::init(argc, argv, "stereo_recorder");
  ros::NodeHandle nh("/");

  ParseCommandLine parser(argv, &whatdoes);

  ArgumentString topic_cam_left(&parser, "l", "left",
                                "sensor_msgs/Image topic for left camera",
                                "/camera_left/image_raw");
  ArgumentString topic_cam_right(&parser, "r", "right",
                                 "sensor_msgs/Image topic for right camera",
                                 "/camera_right/image_raw");
  ArgumentString target_f(&parser, "t", "target",
                          "YAML file containing target metadata", "");
  ArgumentString output_dir(&parser, "o", "output-dir",
                            "Output destination folder", ".");
  ArgumentFlag use_approx_sync(
      &parser, "a", "approx-sync",
      "Use ApproximateTime synchronization policy instead of ExactTime policy");

  parser.parse();

  if (!output_dir.isSet()) {
    std::cerr << "No output destination set.\n"
              << parser.options() << std::endl;
    return -1;
  }

  spdlog::info("Creating output directory " + output_dir.value());
  left_output_dir = std::filesystem::path(output_dir.value()) / "left";
  right_output_dir = std::filesystem::path(output_dir.value()) / "right";
  std::filesystem::create_directories(left_output_dir);
  std::filesystem::create_directories(right_output_dir);

  if (!target_f.isSet()) {
    spdlog::warn("No target provided, target detection will not be available.");
  } else {
    left_target_ptr = ca2lib::TargetBase::fromFile(target_f.value());
    right_target_ptr = ca2lib::TargetBase::fromFile(target_f.value());
  }

  message_filters::Subscriber<sensor_msgs::Image> sub_left(
      nh, topic_cam_left.value(), 10);
  message_filters::Subscriber<sensor_msgs::Image> sub_right(
      nh, topic_cam_right.value(), 10);

  if (use_approx_sync.isSet()) {
    spdlog::info("Using ApproxTime synchronization policy");
    using SyncPolicy =
        message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image>;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_left,
                                                   sub_right);
    sync.registerCallback(std::bind(&sync_callback, std::placeholders::_1,
                                    std::placeholders::_2));
    ros::spin();
  } else {
    spdlog::info("Using ExactTime synchronization policy");
    using SyncPolicy =
        message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                  sensor_msgs::Image>;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_left,
                                                   sub_right);
    sync.registerCallback(std::bind(&sync_callback, std::placeholders::_1,
                                    std::placeholders::_2));
    ros::spin();
  }

  return 0;
}

void sync_callback(const sensor_msgs::ImageConstPtr& left_msg,
                   const sensor_msgs::ImageConstPtr& right_msg) {
  const cv::Mat left_frame = cv_bridge::toCvShare(left_msg, "bgr8")->image;
  const cv::Mat right_frame = cv_bridge::toCvShare(right_msg, "bgr8")->image;

  if (left_detection_mask.empty() or right_detection_mask.empty()) {
    left_detection_mask = cv::Mat(left_frame.size(), CV_8UC3);
    right_detection_mask = cv::Mat(right_frame.size(), CV_8UC3);
    left_detection_mask = cv::Vec3b(255, 255, 255);
    right_detection_mask = cv::Vec3b(255, 255, 255);
  }

  cv::Mat viz_left_frame = left_frame.clone();
  cv::Mat viz_right_frame = right_frame.clone();

  cv::Mat frame_viz, frame_viz_masks;
  cv::hconcat(viz_left_frame, viz_right_frame, frame_viz);
  cv::hconcat(left_detection_mask, right_detection_mask, frame_viz_masks);
  cv::resize(frame_viz, frame_viz,
             cv::Size(frame_viz.cols / 2, frame_viz.rows / 2));
  cv::resize(frame_viz_masks, frame_viz_masks,
             cv::Size(frame_viz_masks.cols / 2, frame_viz_masks.rows / 2));

  cv::imshow("Stereo", frame_viz);
  cv::imshow("Detections", frame_viz_masks);
  char key = cv::waitKey(10);

  const auto ts = left_msg->header.stamp.toSec();
  switch (key) {
    case 'r':
      if (right_target_ptr->detect(right_frame)) {
        right_target_ptr->drawDetection(right_detection_mask);
        std::filesystem::path dest =
            right_output_dir / (std::to_string(ts) + ".png");
        spdlog::info("Saving " + dest.string());
        cv::imwrite(dest.string(), right_frame);
      } else {
        spdlog::warn("Could not detect target on right frame");
      }
      break;
    case 'l':
      if (left_target_ptr->detect(left_frame)) {
        left_target_ptr->drawDetection(left_detection_mask);
        std::filesystem::path dest =
            left_output_dir / (std::to_string(ts) + ".png");
        spdlog::info("Saving " + dest.string());
        cv::imwrite(dest.string(), left_frame);
      } else {
        spdlog::warn("Could not detect target on right frame");
      }
      break;
    case 0x20:  // Space
      if (right_target_ptr->detect(right_frame) &&
          left_target_ptr->detect(left_frame)) {
        right_target_ptr->drawDetection(right_detection_mask);
        left_target_ptr->drawDetection(left_detection_mask);
        std::filesystem::path dest_r =
            right_output_dir / (std::to_string(ts) + ".png");
        std::filesystem::path dest_l =
            left_output_dir / (std::to_string(ts) + ".png");
        spdlog::info("Saving " + dest_r.string() + " | " + dest_l.string());
        cv::imwrite(dest_l.string(), left_frame);
        cv::imwrite(dest_r.string(), right_frame);
      } else {
        spdlog::warn("Could not detect target on both frames");
      }
      break;
    case 0x1B:  // Esc
      spdlog::info("Closing the program");
      ros::shutdown();
      break;
    case 'h':
      std::cout << "press 'r' for right detection only, 'l' for left detection "
                   "only, space for synchronous detection, esc to close the "
                   "program, 'h' to print this line"
                << std::endl;
      break;
  }
}