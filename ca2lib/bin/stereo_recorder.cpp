#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <spdlog/spdlog.h>

#include "ca2lib/parse_command_line.h"
#include "ca2lib/targets.h"

using namespace srrg2_core;

const char* whatdoes =
    "Record images from two cameras. Can be used to store images for "
    "calibration";

void sync_callback(const sensor_msgs::ImageConstPtr& left_msg,
                   const sensor_msgs::ImageConstPtr& right_msg);

ca2lib::TargetBase::SharedPtr target_ptr = nullptr;

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

  if (!target_f.isSet()) {
    spdlog::warn("No target provided, target detection will not be available.");
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
                   const sensor_msgs::ImageConstPtr& right_msg) {}