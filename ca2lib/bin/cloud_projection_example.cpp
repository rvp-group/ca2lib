#include <ros/ros.h>

#include <opencv2/highgui.hpp>

#include "ca2lib/io/ros.h"
#include "ca2lib/projection.h"
#include "ca2lib/types.h"

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg_);

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_projection_example");

  ros::NodeHandle nh;

  ros::Subscriber sub_cloud =
      nh.subscribe<sensor_msgs::PointCloud2>("/cloud_in", 10, cloud_cb);

  ros::spin();
  return 0;
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg_) {
  ca2lib::PointCloudXf cloud = ca2lib::convertRosToPointCloud(msg_);

  const auto lut = ca2lib::projectLidarLUT(cloud);
  const auto ambient_image = ca2lib::composeChannelImage(
      cloud, lut, "ambient", ca2lib::NormalizationType::MINMAX, 0.0f, 200.f);
  const auto intensity_image = ca2lib::composeChannelImage(
      cloud, lut, "intensity", ca2lib::NormalizationType::MINMAX, 0.0f, 200.f);
  const auto range_image = ca2lib::composeChannelImage(
      cloud, lut, "range", ca2lib::NormalizationType::MINMAX, 0.0f, 10000.f);

  cv::imshow("Ambient", ambient_image);
  cv::imshow("Intensity", intensity_image);
  cv::imshow("Range", range_image);
  cv::waitKey(10);
}