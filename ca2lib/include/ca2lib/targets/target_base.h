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

#pragma once
#include <memory>
#include "ca2lib/types.h"

namespace ca2lib {
class TargetBase {
 public:
  using SharedPtr = std::shared_ptr<TargetBase>;

  /**
   * @brief Attempts target detection on the input image. In case target is
   * detected, stores detection parameters internally.
   *
   * @param frame_ Input image
   * @return true if target is detected
   * @return false if target is not detected
   */
  virtual bool detect(const cv::Mat& frame_) = 0;

  /**
   * @brief Attempts target detection on the input image and estimates relative
   * target position estimate. In case target is detected, stores detection and
   * relative pose parameters internally.
   *
   * @param frame_ Input image
   * @param K_ 3x3 Camera matrix
   * @param dist_coeffs_ 1xM distortion parameters
   * @return true if target is detected
   * @return false if target is not detected
   */
  virtual bool detectAndCompute(const cv::Mat& frame_, const cv::Mat& K_,
                                const cv::Mat& dist_coeffs_) = 0;

  /**
   * @brief Returns the detected target corners pixels
   *
   * @return const std::vector<cv::Point2f>& const reference to detected corners
   */
  const std::vector<cv::Point2f>& getCorners() const { return _corners; }

  /**
   * @brief [Available for ChArUco or similar] Returns the detected target
   * corners indices
   *
   * @return const std::vector<int>&
   */
  const std::vector<int>& getCornersIds() const { return _corners_idx; }

  /**
   * @brief [Available for Checkerboard] Returns the associated 3D grid points
   * in target frame
   *
   * @return const std::vector<cv::Point3f>&
   */
  const std::vector<cv::Point3f>& getGridPoints() const { return _grid_points; }

  /**
   * @brief Sets the resize factor
   *
   * @param resize_factor_ desired resize factor (ideally resize_factor <= 1.0)
   */
  inline void setResizeFactor(const float& resize_factor_) {
    _resize_factor = resize_factor_;
    _iresize_factor = 1.0f / _resize_factor;
  }

  /**
   * @brief Draw the detected target in frame
   *
   * @param frame_ Output image (CV_8UC1 or CV_8UC3)
   */
  virtual void drawDetection(cv::Mat& frame_) const = 0;

  /**
   * @brief Returns orientation of target with respect to camera optical
   * frame
   *
   * @return const cv::Mat& orientation in Rodrigues form
   */
  const inline cv::Mat& rvec() const { return _rvec; }

  /**
   * @brief Returns translation of target with respect to camera optical
   * frame
   *
   * @return const cv::Mat& translation vector
   */
  const inline cv::Mat& tvec() const { return _tvec; }

  /**
   * @brief Instantiate a Target based on the input JSON configuration file
   *
   * @param filename_
   * @return TargetBase::SharedPtr
   */
  static TargetBase::SharedPtr fromJson(const std::string&& filename_);

 protected:
  // Frame resize factor. Image can be resized before the detection phase if
  // it's too large.
  float _resize_factor = 1.0, _iresize_factor = 1.0;

  // Transform camera_opt_T_target in Rodrigues (CV) format
  cv::Mat _rvec, _tvec;

  // Detection results
  std::vector<cv::Point2f> _corners;
  std::vector<int> _corners_idx;
  std::vector<cv::Point3f> _grid_points;
};
}  // namespace ca2lib