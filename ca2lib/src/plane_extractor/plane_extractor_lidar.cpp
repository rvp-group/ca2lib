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

#include "ca2lib/plane_extractor/plane_extractor_lidar.h"
#include <numeric>
#include <algorithm>
#include <random>
#include <iostream>

namespace ca2lib {

PlaneType PlaneExtractorLidar::fitPlaneSVD(
    const std::vector<unsigned int>& point_indices_) {
  Eigen::Matrix<float, -1, 3> points =
      Eigen::Matrix<float, -1, 3>(point_indices_.size(), 3);

  for (unsigned int i = 0; i < point_indices_.size(); ++i) {
    // Extract XYZ components from input cloud
    points.row(i) = _cloud->points[point_indices_[i]].head<3>();
  }

  const auto mean = points.colwise().mean();
  const auto centered = points.rowwise() - mean;
  const Eigen::MatrixXf cov =
      (centered.transpose() * centered) / float(points.rows() - 1);

  Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::ComputeThinV);
  Eigen::Vector3f normal = svd.matrixV().rightCols<1>();
  float c = -mean.dot(normal);
  if (c < 0) {
    normal = -normal;
    c = -c;
  }
  PlaneType plane;
  plane << normal, c;
  return plane;
}

bool PlaneExtractorLidar::process() {
  _inliers.clear();
  // Initialize sampling bucket
  std::vector<unsigned int> valid_points;
  if (_mask.size())
    valid_points = _mask;
  else {
    valid_points.resize(_cloud->points.size());
    std::iota(valid_points.begin(), valid_points.end(), 0);
  }

  unsigned int num_points = valid_points.size();
  std::vector<unsigned int> it_inliers;
  it_inliers.reserve(num_points);
  for (unsigned int it = 0; it < _ransac_params.num_iterations; ++it) {
    it_inliers.clear();

    // Sample 3 points
    std::vector<unsigned int> sample;
    std::sample(valid_points.begin(), valid_points.end(),
                std::back_inserter(sample), 3,
                std::mt19937{std::random_device{}()});
    // Compute model for current iteration
    PlaneType it_model = fitPlaneSVD(sample);

    // Eval solution
    Eigen::Vector3f normal = -it_model.head<3>();
    Eigen::Vector3f point_in_plane = normal * it_model.w();

    for (const auto& pidx : valid_points) {
      const Eigen::Vector3f p = _cloud->points[pidx].head<3>();
      float error = (p - point_in_plane).dot(normal);
      error = error * error;
      if (error <
          _ransac_params.max_error_thresh * _ransac_params.max_error_thresh)
        it_inliers.push_back(pidx);
    }

    float perc_inliers =
        (float)(it_inliers.size()) / (float)(valid_points.size());

    // Discard bad solutionss
    if (perc_inliers < _ransac_params.min_inliers_thresh) continue;

    if (it_inliers.size() > _inliers.size()) {
      _inliers = it_inliers;
    }
  }
  if (_inliers.size()) {
    _plane = fitPlaneSVD(_inliers);
  }
  return _inliers.size();
}
}  // namespace ca2lib