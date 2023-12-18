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

#include "ca2lib/projection.h"
#include <numeric>
#include <iostream>

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
    const float az = atan2f(p(1), p(0));
    const int col = cvRound(fx * az + (cols * 0.5f));
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

}  // namespace ca2lib