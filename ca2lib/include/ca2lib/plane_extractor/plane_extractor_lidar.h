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

#pragma once
#include "ca2lib/types.h"
#include "plane_extractor_base.h"

namespace ca2lib {
class PlaneExtractorLidar : public PlaneExtractorBase {
 public:
  /**
   * @brief RANSAC parameters for plane search.
   * -num_iterations   :         No. of RANSAC iterations
   * -max_error_thresh :         Maximum allowed error for a point to be
   * -considered inlier min_inlier_thresh:  Minimum number of inliers in
   * percentage for a solution to be considered valid
   */
  struct RansacParams {
    unsigned int num_iterations = 100;
    float max_error_thresh = 0.1;
    float min_inliers_thresh = 0.6;
  };

  /**
   * @brief Set the RANSAC parameters. These will be used to estimate the plane
   * inside the cloud.
   *
   * @param params_
   */
  inline void setRansacParams(const RansacParams& params_) {
    _ransac_params = params_;
  }

  /**
   * @brief Load the input cloud
   *
   * @param cloud_
   */
  inline void setData(const PointCloudXf* cloud_) { _cloud = cloud_; }

  inline std::vector<unsigned int>& mask() { return _mask; }
  inline const std::vector<unsigned int>& mask() const { return _mask; }

  inline const std::vector<unsigned int>& inliers() const { return _inliers; }

  /**
   * @brief Interface for PlaneExtractors. When called, the extractor process
   * input data to find the target plane.
   *
   * @return true plane was found
   * @return false otherwise
   */
  bool process() override;

 protected:
  /**
   * @brief Given a set of indices (related to _cloud object), estimates a plane
   * using SVD decomposition.
   *
   * @param points_indices
   * @return Plane
   */
  Plane fitPlaneSVD(const std::vector<unsigned int>& points_indices);

  const PointCloudXf* _cloud;
  RansacParams _ransac_params;

  std::vector<unsigned int> _inliers;
  std::vector<unsigned int> _mask;
};

}  // namespace ca2lib
