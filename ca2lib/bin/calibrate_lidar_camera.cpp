#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <opencv2/highgui.hpp>

#include "ca2lib/io/ros.h"
#include "ca2lib/parse_command_line.h"
#include "ca2lib/plane_extractor.h"
#include "ca2lib/projection.h"
#include "ca2lib/targets.h"
#include "ca2lib/types.h"

using namespace srrg2_core;

const char* whatdoes = "Calibrate LiDAR-RGB Camera pair.";

cv::Mat viewport;
ca2lib::CameraIntrinsics camera_info;
ca2lib::TargetBase::SharedPtr target_ptr = nullptr;
// Range Image max range
float ri_max_range = 10000;
bool roi_set = false;
int roi_radius;
bool plane_found = false;
cv::Point2i roi_center;
cv::Point2i selection_center;
int selection_radius = 10;
ca2lib::PlaneExtractorLidar plane_extractor;
std::vector<cv::Point> ransac_inlier_pixels;
cv::Mat range_image, camera_image;

void data_callback(const sensor_msgs::PointCloud2::ConstPtr&,
                   const sensor_msgs::Image::ConstPtr&);
void updateViewport(int);
void mouseCallback(int, int, int, int, void*);

int main(int argc, char** argv) {
  ros::init(argc, argv, "calibrate_lidar_camera");

  ParseCommandLine parser(argv, &whatdoes);
  ArgumentString camera_intrinsics_f(&parser, "i", "intrinsics",
                                     "Camera intrinsics file", "");
  ArgumentString output_f(&parser, "o", "output",
                          "Output calibration file [ends for .yaml or .json]",
                          "lidar_in_camera.yaml");
  ArgumentString target_f(&parser, "t", "target",
                          "YAML file containing target metadata", "");

  ArgumentString topic_cam(&parser, "c", "topic-camera",
                           "sensor_msgs/Image topic for camera",
                           "/camera_right/image_raw");
  ArgumentString topic_cloud(&parser, "l", "topic-cloud",
                             "sensor_msgs/PointCloud2 topic for lidar",
                             "/ouster/points");
  ArgumentFlag use_approx_sync(
      &parser, "a", "approx-sync",
      "Use ApproximateTime synchronization policy instead of ExactTime policy");

  parser.parse();

  if (!camera_intrinsics_f.isSet()) {
    std::cerr << "No camera intrinsics file set.\n"
              << parser.options() << std::endl;
    return -1;
  }

  if (!target_f.isSet()) {
    std::cerr << "No target file set.\n" << parser.options() << std::endl;
    return -1;
  }

  spdlog::info("Reading target config " + target_f.value());
  target_ptr = ca2lib::TargetBase::fromFile(target_f.value());

  spdlog::info("Reading camera intrinsics " + camera_intrinsics_f.value());
  camera_info = ca2lib::CameraIntrinsics::load(camera_intrinsics_f.value());

  cv::namedWindow("Viewport");
  cv::setMouseCallback("Viewport", mouseCallback);

  plane_extractor.setRansacParams({300, 0.02, 0.1});

  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud(
      nh, topic_cloud.value(), 10);
  message_filters::Subscriber<sensor_msgs::Image> sub_image(
      nh, topic_cam.value(), 10);

  if (use_approx_sync.isSet()) {
    spdlog::info("Using ApproxTime synchronization policy");
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::PointCloud2, sensor_msgs::Image>;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_cloud,
                                                   sub_image);
    sync.registerCallback(std::bind(&data_callback, std::placeholders::_1,
                                    std::placeholders::_2));
    while (ros::ok()) {
      ros::spinOnce();
      if (!viewport.empty()) cv::imshow("Viewport", viewport);
      updateViewport(cv::waitKey(10));
    }
  } else {
    spdlog::info("Using ExactTime synchronization policy");
    using SyncPolicy =
        message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2,
                                                  sensor_msgs::Image>;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_cloud,
                                                   sub_image);
    sync.registerCallback(std::bind(&data_callback, std::placeholders::_1,
                                    std::placeholders::_2));
    while (ros::ok()) {
      ros::spinOnce();
      if (!viewport.empty()) {
        cv::imshow("Camera Image", camera_image);
        cv::imshow("Viewport", viewport);
      }
      updateViewport(cv::waitKey(10));
    }
  }

  return 0;
}

void data_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_,
                   const sensor_msgs::Image::ConstPtr& image_msg_) {
  // Process Cloud
  auto cloud = ca2lib::convertRosToPointCloud(cloud_msg_);
  ca2lib::InverseLut_t inverse_lut;
  const auto lut = ca2lib::projectLidarLUT(cloud, inverse_lut);

  // Extract data from ROS messages
  range_image = ca2lib::composeChannelImage(
      cloud, lut, "range", ca2lib::NormalizationType::MINMAX, 0, ri_max_range);

  camera_image = cv_bridge::toCvShare(image_msg_, "bgr8")->image;

  // Find target on LiDAR cloud
  if (roi_set) {
    // Create ROI mask
    std::vector<unsigned int> train_points;
    train_points.reserve(cloud.points.size());
    cv::Mat mask = cv::Mat::zeros(range_image.size(), CV_8UC1);
    cv::circle(mask, roi_center, roi_radius, 1, -1);
    // Store masked cloud points using LUT
    for (int r = -roi_radius; r < roi_radius; ++r) {
      for (int c = -roi_radius; c < roi_radius; ++c) {
        const cv::Point2i query(roi_center.x + r, roi_center.y + c);
        if (query.x < 0 or query.x >= mask.cols or query.y < 0 or
            query.y >= mask.rows) {
          continue;
        }
        if (mask.at<unsigned char>(query) > 0) {
          int32_t lut_idx = lut.at<int32_t>(query);
          if (lut_idx == -1) continue;
          train_points.push_back(lut_idx);
        }
      }
    }
    // Run plane extractor
    if (train_points.size() > 3) {
      plane_extractor.mask() = train_points;
      plane_extractor.setData(&cloud);
      plane_found = plane_extractor.process();
    } else
      plane_found = false;
  }

  if (plane_found) {
    ransac_inlier_pixels.clear();
    ransac_inlier_pixels.reserve(plane_extractor.inliers().size());
    for (const auto idx : plane_extractor.inliers()) {
      const auto& [valid, p] = inverse_lut[idx];
      if (valid) {
        ransac_inlier_pixels.push_back(p);
      }
    }
  }
}

void updateViewport(int key_pressed) {
  if (range_image.empty() or camera_image.empty()) return;
  cv::cvtColor(range_image, viewport, cv::COLOR_GRAY2BGR);
  if (plane_found) {
    // Draw RANSAC inliers
    for (const auto p : ransac_inlier_pixels) {
      viewport.at<cv::Vec3f>(p) = cv::Vec3f(0, 0, 1.);
    }
  }

  cv::circle(viewport, selection_center, selection_radius,
             cv::Scalar(125, 125, 125), 1);
  cv::circle(viewport, roi_center, roi_radius, cv::Scalar(0, 255, 0), 1);
  switch (key_pressed) {
    case 0x69:  // i
      selection_radius++;
      break;
    case 0x6F:  // o
      selection_radius--;
      break;

    case 0x2B:  // +
      ri_max_range += 1000;
      break;

    case 0x2D:  // -
      ri_max_range = std::max(1000., ri_max_range - 1000.);
      break;

    case 0x0D:  // Enter
      if (target_ptr->detectAndCompute(camera_image, camera_info.K,
                                       camera_info.dist_coeffs)) {
        spdlog::info("Target detected on camera image");
      }
      break;

    default:
      break;
  }
}

void mouseCallback(int event, int x, int y, int flags, void*) {
  selection_center = cv::Point2i(x, y);
  if (event == cv::EVENT_LBUTTONDOWN) {
    roi_set = true;
    roi_center = selection_center;
    roi_radius = selection_radius;
  }
}