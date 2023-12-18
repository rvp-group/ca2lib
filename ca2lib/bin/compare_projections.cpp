#include <ros/ros.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "ca2lib/io/ros.h"
#include "ca2lib/projection.h"
#include "ca2lib/types.h"

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg_);

int main(int argc, char** argv) {
  ros::init(argc, argv, "compare_proections");

  ros::NodeHandle nh;

  ros::Subscriber sub_cloud =
      nh.subscribe<sensor_msgs::PointCloud2>("/cloud_in", 10, cloud_cb);

  ros::spin();
  return 0;
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg_) {
  ca2lib::PointCloudXf cloud = ca2lib::convertRosToPointCloud(msg_);

  const auto lut_by_id = ca2lib::projectLidarLUT(cloud);
  const auto lut_by_ea =
      ca2lib::projectSphericalLidarLUT(cloud, M_PI * 2, M_PI_2, 128, 1024);

  const auto intensity_by_id = ca2lib::composeChannelImage(
      cloud, lut_by_id, "intensity", ca2lib::NormalizationType::MINMAX, 0, 200);
  const auto intensity_by_ea = ca2lib::composeChannelImage(
      cloud, lut_by_ea, "intensity", ca2lib::NormalizationType::MINMAX, 0, 200);

  auto ring_by_id = ca2lib::composeChannelImage(
      cloud, lut_by_id, "ring", ca2lib::NormalizationType::MINMAX, 0, 127);
  auto ring_by_ea = ca2lib::composeChannelImage(
      cloud, lut_by_ea, "ring", ca2lib::NormalizationType::MINMAX, 0, 127);
  cv::Mat mask_ring_id, mask_ring_ea;
  mask_ring_id = lut_by_id == -1;
  mask_ring_ea = lut_by_ea == -1;

  cv::Mat pbid_intensity, pbea_intensity;
  cv::convertScaleAbs(intensity_by_id, pbid_intensity, 255, 0);
  cv::convertScaleAbs(intensity_by_ea, pbea_intensity, 255, 0);

  cv::convertScaleAbs(ring_by_id, ring_by_id, 255, 0);
  cv::convertScaleAbs(ring_by_ea, ring_by_ea, 255, 0);

  cv::applyColorMap(ring_by_id, ring_by_id, cv::COLORMAP_RAINBOW);
  cv::applyColorMap(ring_by_ea, ring_by_ea, cv::COLORMAP_RAINBOW);

  ring_by_id.setTo(cv::Scalar(0, 0, 0), mask_ring_id);
  ring_by_ea.setTo(cv::Scalar(0, 0, 0), mask_ring_ea);

  cv::imshow("mask_id", mask_ring_id);

  cv::imshow("Projection by ID", pbid_intensity);
  cv::imshow("Projection by Elevation Angle", pbea_intensity);

  cv::imshow("Ring| Projection by ID", ring_by_id);
  cv::imshow("Ring| Projection by Elevation Angle", ring_by_ea);

  cv::imwrite("/home/eg/Desktop/pbid_intensity.png", pbid_intensity);
  cv::imwrite("/home/eg/Desktop/pbid_ring.png", ring_by_id);
  cv::imwrite("/home/eg/Desktop/pbea_intensity.png", pbea_intensity);
  cv::imwrite("/home/eg/Desktop/pbea_ring.png", ring_by_ea);
  cv::waitKey(10);
  //   const auto ambient_image = ca2lib::composeChannelImage(
  //       cloud, lut, "ambient", ca2lib::NormalizationType::MINMAX, 0.0f,
  //       200.f);
  //   const auto intensity_image = ca2lib::composeChannelImage(
  //       cloud, lut, "intensity", ca2lib::NormalizationType::MINMAX, 0.0f,
  //       200.f);
  //   const auto range_image = ca2lib::composeChannelImage(
  //       cloud, lut, "range", ca2lib::NormalizationType::MINMAX, 0.0f,
  //       10000.f);

  //   cv::imshow("Ambient", ambient_image);
  //   cv::imshow("Intensity", intensity_image);
  //   cv::imshow("Range", range_image);
  //   cv::waitKey(10);
}