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
 * @param hfov [Optional] Horizontal Field of View of the LiDAR (assumed 2*pi)
 * @param rows [Optional] Number of rows in the lut table (equal to no. rings)
 * @param cols [Optional] Number of columns in the lut table
 * @return cv::Mat LUT table
 */
cv::Mat projectLidarLUT(const PointCloudXf& cloud_in, const float hfov_,
                        unsigned int rows_, unsigned int cols_) {
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

  const float fx = cols / hfov;

  // Create and fill LookUpTable
  cv::Mat lut = cv::Mat_<int32_t>(rows, cols);
  lut = -1;
  for (unsigned int i = 0; i < cloud_in.points.size(); ++i) {
    const auto& p = cloud_in.points[i];
    int row = i / cols;
    if (contains_ring) row = p(fields.at("ring").first);
    const float az = atan2f(p(1), p(0));
    const int col = cvRound(fx * az + (cols * 0.5f));
    lut.at<int32_t>(row, col) = i;
  }

  return lut;
}

cv::Mat composeChannelImage(const PointCloudXf& cloud_in,
                            const cv::Mat& lut_table,
                            const std::string& channel,
                            const float norm_factor = 0.0f) {
  // TODO
  return cv::Mat();
}

}  // namespace ca2lib