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

#include "ca2lib/projection.h"
#include <numeric>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>

namespace ca2lib {

/**
 * @brief Computes projection indices for all the points of cloud_in and returns
 * a lookup table. The lookup table contains an index for every pixel. The index
 * may be invalid (-1) or, a positive integer that represents the index of the
 * point in cloud_in that is projected on that pixel
 *
 * The size of the lookup table is automatically infered by the ring field or by
 * the cloud width/height. If these informations are not available, input rows
 * and cols are used otherwise.
 *
 * @param cloud_in Input cloud
 * @param inverse_lut Inverse lookup table: contains coordinate of projection
 *                      for each point
 * @param hfov [Optional] Horizontal Field of View of the LiDAR (assumed 2*pi)
 * @param rows [Optional] Number of rows in the lut table (equal to no. rings)
 * @param cols [Optional] Number of columns in the lut table
 * @return cv::Mat LUT table
 */
cv::Mat projectLidarLUT(const PointCloudXf& cloud_in,
                        std::vector<std::pair<bool, cv::Point2i>>& inverse_lut,
                        const float hfov_, unsigned int rows_,
                        unsigned int cols_) {
  const auto& fields = cloud_in.fields;

  bool contains_ring = fields.find("ring") != fields.end();
  bool valid_wh = cloud_in.width > 1 and cloud_in.height > 1;

  unsigned int rows = rows_;
  unsigned int cols = cols_;
  const float hfov = hfov_ != 0.0f ? hfov_ : 2 * M_PI;

  // Infer LUT size
  if (contains_ring) {
    const auto ring_idx = fields.at("ring").first;
    const auto p_max =
        *std::max_element(cloud_in.points.begin(), cloud_in.points.end(),
                          [ring_idx](Eigen::VectorXf a, Eigen::VectorXf b) {
                            return a(ring_idx) < b(ring_idx);
                          });
    rows = p_max(ring_idx) + 1;
    cols = cloud_in.points.size() / rows;
  } else if (valid_wh) {
    rows = cloud_in.height;
    cols = cloud_in.width;
  }

  const float fx = -(int)cols / hfov;

  // Create and fill LookUpTable
  inverse_lut.clear();
  inverse_lut.reserve(cloud_in.points.size());
  cv::Mat lut = cv::Mat_<int32_t>(rows, cols);
  lut = -1;
  for (unsigned int i = 0; i < cloud_in.points.size(); ++i) {
    const auto& p = cloud_in.points[i];
    if (p.head<3>().isApprox(Eigen::Vector3f::Zero())) {
      inverse_lut.emplace_back(false, cv::Point2i(0, 0));
      continue;
    }
    int row = i / cols;
    if (contains_ring) row = p(fields.at("ring").first);
    row = std::max(0, std::min(row, static_cast<int>(rows - 1)));

    const float az = atan2f(p(1), p(0));
    int col = cvRound(fx * az + (cols * 0.5f));
    if (col < 0 || col >= cols) {
      // ignore and mark as invalid
      inverse_lut.emplace_back(false, cv::Point2i(0, 0)); 
      continue;
    }

    lut.at<int32_t>(row, col) = i;
    inverse_lut.emplace_back(true, cv::Point2i(col, row));
  }

  return lut;
}

/**
 * @brief Computes projection indices for all the points of cloud_in and returns
 * a lookup table. The lookup table contains an index for every pixel. The index
 * may be invalid (-1) or, a positive integer that represents the index of the
 * point in cloud_in that is projected on that pixel
 *
 * The size of the lookup table is automatically infered by the ring field or by
 * the cloud width/height. If these informations are not available, input rows
 * and cols are used otherwise.
 *
 * This function is an overload that does not expose the inverse_lut.
 *
 * @param cloud_in Input cloud
 * @param hfov [Optional] Horizontal Field of View of the LiDAR (assumed 2*pi)
 * @param rows [Optional] Number of rows in the lut table (equal to no. rings)
 * @param cols [Optional] Number of columns in the lut table
 * @return cv::Mat LUT table
 */
cv::Mat projectLidarLUT(const PointCloudXf& cloud_in, const float hfov_,
                        unsigned int rows_, unsigned int cols_) {
  std::vector<std::pair<bool, cv::Point2i>> inv_lut;
  return projectLidarLUT(cloud_in, inv_lut, hfov_, rows_, cols_);
}

/**
 * @brief Composes an image using Projection by ID of a given cloud's channel.
 *
 * @param cloud_in Input cloud
 * @param lut_table LookUp Table computed with projectLidarLUT
 * @param channel channel to render
 * @param norm_type type of normalization technique
 * @param a first normalization factor
 * @param b second normalization factor
 * @return cv::Mat
 */
cv::Mat composeChannelImage(const PointCloudXf& cloud_in,
                            const cv::Mat& lut_table,
                            const std::string& channel,
                            const NormalizationType norm_type, const float a_,
                            const float b_) {
  const auto& fields = cloud_in.fields;
  const auto& points = cloud_in.points;
  if (fields.find(channel) == fields.end())
    throw std::runtime_error("Input channel does not exists");

  const auto field_idx = fields.at(channel).first;

  float a = a_;
  float b = b_;

  switch (norm_type) {
    case UNCHANGED:
      a = 1.0;
      b = 0.0;
      break;
    case NORMALIZE: {
      const auto& p_min =
          *std::min_element(points.begin(), points.end(),
                            [field_idx](Eigen::VectorXf a, Eigen::VectorXf b) {
                              return a(field_idx) < b(field_idx);
                            });
      const auto& p_max =
          *std::max_element(points.begin(), points.end(),
                            [field_idx](Eigen::VectorXf a, Eigen::VectorXf b) {
                              return a(field_idx) < b(field_idx);
                            });
      b = -p_min(field_idx);
      a = 1.0f / p_max(field_idx);
      break;
    }
    case MINMAX: {
      // a_ is minimum value
      // b_ is maximum value
      b = -a_;
      a = 1.0 / b_;
      break;
    }
  }

  cv::Mat_<float> channel_image(lut_table.size());
  channel_image = 0.0f;

  for (int r = 0; r < lut_table.rows; ++r) {
    for (int c = 0; c < lut_table.cols; ++c) {
      const auto idx = lut_table.at<int32_t>(r, c);
      if (idx >= 0) {
        const float val =
            (points[lut_table.at<int32_t>(r, c)](field_idx) + b) * a;
        channel_image.at<float>(r, c) = std::min(std::max(0.0f, val), 1.0f);
      }
    }
  }
  return channel_image;
}

cv::Mat projectSphericalLidarLUT(const PointCloudXf& cloud_in, const float hfov,
                                 const float vfov, unsigned int rows,
                                 unsigned int cols) {
  cv::Mat lut = cv::Mat_<int32_t>(rows, cols);
  lut = -1;

  const float fx = -(int)cols / hfov;
  const float fy = -(int)rows / vfov;
  const float cx = cols / 2;
  const float cy = rows / 2;

  for (unsigned int i = 0; i < cloud_in.points.size(); ++i) {
    const auto& p = cloud_in.points[i];
    if (p.head<3>().isApprox(Eigen::Vector3f::Zero())) continue;
    const float az = atan2f(p(1), p(0));
    const float el = atan2(p(2), sqrtf(p(0) * p(0) + p(1) * p(1)));

    const int col = cvRound(fx * az + cx);
    const int row = cvRound(fy * el + cy);
    if (row < 0 or row >= rows or col < 0 or col >= cols) continue;
    lut.at<int32_t>(row, col) = i;
  }
  return lut;
}

/**
 * @brief Computes the PINHOLE projection indices for all the points of cloud_in
 * and returns a lookup table. The Lookup table contains an index for
 * every pixel. THe index may be invalid (-1) or, a positive integer that
 * represents the index of the point in cloud_in that is projected on that pixel
 *
 * @param cloud_in Input cloud
 * @param inverse_lut Inverse lookup table: contains coordinate of projection
 *                      for each point
 * @param image_size Size of the lookup table
 * @param camera_intrinsics camera intrinsics (K, dist_coeffs)
 * @param camera_T_cloud Isometry that maps points in cloud frame to camera
 * frame (i.e. p_cam = camera_T_cloud * p_cloud)
 * @return cv::Mat
 */
cv::Mat projectPinholeLUT(
    const PointCloudXf& cloud_in,
    std::vector<std::pair<bool, cv::Point2i>>& inverse_lut,
    const cv::Size image_size, const CameraIntrinsics camera_intrinsics,
    const Eigen::Isometry3f& camera_T_cloud) {
  inverse_lut.clear();
  inverse_lut.resize(cloud_in.points.size());

  const auto& fields = cloud_in.fields;

  std::vector<int32_t> points_indices;
  std::vector<cv::Vec3f> points_cv;
  for (unsigned int i = 0; i < cloud_in.points.size(); ++i) {
    const auto& p = cloud_in.points[i];
    const auto p_cam = camera_T_cloud * p.head<3>();
    if (p_cam(fields.at("z").first) < 0) {
      inverse_lut[i] = {false, cv::Point2i(0, 0)};
      continue;
    }

    points_indices.push_back(i);
    // points_cv.push_back({p(fields.at("x").first),
    //                      p(fields.at("y").first, p(fields.at("z").first))});
    points_cv.push_back({p_cam.x(), p_cam.y(), p_cam.z()});
  }

  // cv::Mat R, rvec, tvec;
  // cv::eigen2cv((Eigen::Matrix3f)camera_T_cloud.linear(), R);

  // cv::Rodrigues(R, rvec);
  // cv::eigen2cv((Eigen::Vector3f)camera_T_cloud.translation(), tvec);

  cv::Mat rvec(3, 1, CV_32F);
  cv::Mat tvec(3, 1, CV_32F);
  rvec = 0;
  tvec = 0;
  std::vector<cv::Point2f> points_2d;
  if (camera_intrinsics.dist_coeffs.cols == 4) {
    cv::fisheye::projectPoints(points_cv, points_2d, rvec, tvec,
                               camera_intrinsics.K,
                               camera_intrinsics.dist_coeffs);
  } else {
    cv::projectPoints(points_cv, rvec, tvec, camera_intrinsics.K,
                      camera_intrinsics.dist_coeffs, points_2d);
  }

  cv::Mat lut = cv::Mat_<int32_t>(image_size);
  lut = -1;
  for (unsigned int i = 0; i < points_2d.size(); ++i) {
    // TODO: Check coordinate bounds on image_size
    if (points_2d[i].x < 0 or points_2d[i].x >= image_size.width or
        points_2d[i].y < 0 or points_2d[i].y >= image_size.height) {
      inverse_lut[points_indices[i]] = {false, cv::Point2i(0, 0)};
      continue;
    }
    // TODO: Save point_index on point_coordinate
    cv::Point2i index = cv::Point2i(points_2d[i].x, points_2d[i].y);
    lut.at<int32_t>(index) = points_indices[i];
    inverse_lut[points_indices[i]] = {true, index};
  }

  return lut;
}
}  // namespace ca2lib