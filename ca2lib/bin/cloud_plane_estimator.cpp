#include <ros/ros.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "ca2lib/io/ros.h"
#include "ca2lib/plane_extractor.h"
#include "ca2lib/projection.h"
#include "ca2lib/types.h"

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg_);

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_plane_estimator");

  ros::NodeHandle nh;

  ros::Subscriber sub_cloud =
      nh.subscribe<sensor_msgs::PointCloud2>("/cloud_in", 10, cloud_cb);

  ros::spin();
  return 0;
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg_) {
  ca2lib::PointCloudXf cloud = ca2lib::convertRosToPointCloud(msg_);

  std::vector<std::pair<bool, cv::Point2i>> inverse_lut;
  const auto lut = ca2lib::projectLidarLUT(cloud, inverse_lut);
  const auto range_image = ca2lib::composeChannelImage(
      cloud, lut, "range", ca2lib::NormalizationType::MINMAX, 0.0f, 10000.f);

  cv::Rect2i roi = cv::Rect(cv::Point2i(300, 30), cv::Point2i(600, 100));

  cv::Mat lut_roi = lut(roi);

  std::vector<unsigned int> valid_points;
  valid_points.reserve(cloud.points.size());
  // for (unsigned int i = 0; i < cloud.points.size(); ++i) {
  //   if (cloud.points[i].head<3>().isApprox(Eigen::Vector3f::Zero()))
  //   continue; valid_points.push_back(i);
  // }
  for (int r = 0; r < lut_roi.rows; ++r) {
    for (int c = 0; c < lut_roi.cols; ++c) {
      int32_t lut_idx = lut_roi.at<int32_t>(r, c);
      if (lut_idx == -1) continue;
      valid_points.push_back(lut_idx);
    }
  }

  ca2lib::PlaneExtractorLidar extractor;
  extractor.setRansacParams({300, 0.02, 0.1});
  extractor.mask() = valid_points;
  extractor.setData(&cloud);

  extractor.process();

  cv::Mat ransac_image;
  cv::cvtColor(range_image, ransac_image, cv::COLOR_GRAY2BGR);

  for (const auto idx : extractor.inliers()) {
    const auto& [valid, p] = inverse_lut[idx];
    if (valid) {
      ransac_image.at<cv::Vec3f>(p) = cv::Vec3f(0, 0, 1.);
    }
  }
  cv::rectangle(ransac_image, roi, {0, 255, 0}, 2);

  cv::imshow("ransac", ransac_image);
  cv::waitKey(1);

  // for (int r = 0; r < ransac_image.rows; ++r) {
  //   for (int c = 0; c < ransac_image.cols; ++c) {
  //     const auto pixel_idx = lut.at<int32_t>(r, c);
  //     if (pixel_idx == -1) continue;
  //     const auto it = std::find(extractor.inliers().begin(),
  //                               extractor.inliers().end(), pixel_idx);
  //     if (it != extractor.inliers().end()) {
  //       ransac_image.at<cv::Vec3f>(r, c) = cv::Vec3f(0, 0, 1.0);
  //     }
  //   }
  // }
  // std::cerr << "done" << std::endl;

  // cv::imshow("Ransac", ransac_image);
  // cv::waitKey(1);
}