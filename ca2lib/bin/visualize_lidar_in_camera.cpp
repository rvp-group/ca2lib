#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include "ca2lib/io/ros.h"
#include "ca2lib/parse_command_line.h"
#include "ca2lib/projection.h"
#include "ca2lib/targets.h"
#include <spdlog/spdlog.h>
#include "ca2lib/types.h"
#include <opencv2/highgui.hpp>

using namespace srrg2_core;

const char* whatdoes = "Visualize Cloud projection on camera images using the obtained transform camera_T_lidar";
ca2lib::CameraIntrinsics camera_info;
ca2lib::CameraLidarExtrinsics extrinsics;
void data_callback(const sensor_msgs::PointCloud2::ConstPtr&,
                   const sensor_msgs::Image::ConstPtr&);

int main(int argc, char** argv) {
    ros::init(argc, argv, "visualize_lidar_in_camera");

    ParseCommandLine parser(argv, &whatdoes);
    ArgumentString camera_intrinsics_f(&parser, "i", "intrinsics",
                                        "Camera intrinsics file", "");
    ArgumentString lidar_in_camera_f(&parser, "e", "extrinsics", "Input lidar_in_camera file [ends for .yaml or .json]", "");                                        
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
    ArgumentFlag use_approx_sync(&parser, "a", "approx-sync",
                               "Use ApproximateTime synchronization policy "
                               "instead of ExactTime policy");

    parser.parse();
    
    if (!camera_intrinsics_f.isSet()) {
        std::cerr << "No camera intrinsics file set.\n"
                << parser.options() << std::endl;
        return -1;
    }

    if (!lidar_in_camera_f.isSet()) {
        std::cerr << "No camera extrinsics file set.\n" << parser.options() << std::endl;
    }

    camera_info = ca2lib::CameraIntrinsics::load(camera_intrinsics_f.value());
    extrinsics = ca2lib::CameraLidarExtrinsics::load(lidar_in_camera_f.value());
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
        ros::spin();
    } else {
        spdlog::info("Using ExactTime synchronization policy");
        using SyncPolicy =
            message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2,
                                                    sensor_msgs::Image>;
        message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_cloud,
                                                    sub_image);
        sync.registerCallback(std::bind(&data_callback, std::placeholders::_1,
                                        std::placeholders::_2));
        ros::spin();
    }
    
    return 0;
}


cv::Mat projectLidar(const cv::Mat& image_rgb,
                     const ca2lib::PointCloudXf& cloud,
                     const Eigen::Isometry3f& camera_T_lidar) {
  cv::Mat ret = image_rgb.clone();
  ca2lib::InverseLut_t inv_lut;
  ca2lib::projectPinholeLUT(cloud, inv_lut, ret.size(),
                                          camera_info, camera_T_lidar);

  for (unsigned int i = 0; i < cloud.points.size(); ++i) {
    if (inv_lut[i].first)
      cv::circle(ret, inv_lut[i].second, 3, {0, 0, 255}, -1);
  }
  return ret;
}

void data_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_,
                   const sensor_msgs::Image::ConstPtr& image_msg_) {
    ca2lib::PointCloudXf cloud;
    cloud = ca2lib::convertRosToPointCloud(cloud_msg_);
    cv::Mat camera_image = cv_bridge::toCvShare(image_msg_, "bgr8")->image;
    cv::Mat cloud_reprojected = projectLidar(camera_image, cloud, extrinsics.lidar_in_camera); 
    cv::imshow("Reprojection", cloud_reprojected);
    cv::waitKey(10);
}