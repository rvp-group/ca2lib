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

#include "ca2lib/targets/target_checkerboard.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

namespace ca2lib {
/**
 * @brief Attempts target detection on the input image. In case target is
 * detected, stores detection parameters internally.
 *
 * @param frame_ Input image (CV_8UC1 or CV_8UC3)
 * @return true if target is detected
 * @return false if target is not detected
 */
bool TargetCheckerboard::detect(const cv::Mat& frame_) {
  CV_Assert(frame_.type() == CV_8UC1 or frame_.type() == CV_8UC3);
  cv::Mat frame_gray;
  if (frame_.type() == CV_8UC1)
    frame_gray = frame_;
  else
    cv::cvtColor(frame_, frame_gray, cv::COLOR_BGR2GRAY);

  _corners.clear();
  _corners_idx.clear();

  bool target_found = cv::findChessboardCornersSB(
      frame_gray, cv::Size(_cols, _rows), _corners,
      cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_EXHAUSTIVE +
          cv::CALIB_CB_ACCURACY);
  return target_found;
}

/**
 * @brief Attempts target detection on the input image and estimates relative
 * target position estimate. In case target is detected, stores detection and
 * relative pose parameters internally.
 *
 * @param frame_ Input image (CV_8UC1 or CV_8UC3)
 * @param K_ 3x3 Camera matrix (CV_16 or CV_32F or CV_64F)
 * @param dist_coeffs_ 1xM distortion parameters (CV_16 or CV_32 or CV_64F)
 * @return true if target is detected
 * @return false if target is not detected
 */
bool TargetCheckerboard::detectAndCompute(const cv::Mat& frame_,
                                          const cv::Mat& K_,
                                          const cv::Mat& dist_coeffs_) {
  if (!detect(frame_)) return false;
  bool ret = false;
  if (dist_coeffs_.cols == 4) {
    // Equidistant model:
    // Solve using cv::fisheye stuff
    std::vector<cv::Point2f> undistorted_corners;
    cv::fisheye::undistortPoints(_corners, undistorted_corners, K_,
                                 dist_coeffs_);
    cv::Mat identity = cv::Mat::eye(3, 3, K_.type());
    ret = cv::solvePnP(_grid_points, undistorted_corners, identity, cv::Mat(),
                       _rvec, _tvec, false, cv::SOLVEPNP_IPPE);
  } else {
    // rad-tan model:
    // Solve using cv:: stuff
    std::cerr << "k=" << K_ << " dist_coeffs=" << dist_coeffs_ << std::endl;
    ret = cv::solvePnP(_grid_points, _corners, K_, dist_coeffs_, _rvec, _tvec,
                       false, cv::SOLVEPNP_IPPE);
    std::cerr << "computing rvec=" << _rvec << " " << _tvec << std::endl;
  }
  return ret;
}

/**
 * @brief Draw the detected target in frame
 *
 * @param frame_ Output image (CV_8UC1 or CV_8UC3)
 */
void TargetCheckerboard::drawDetection(cv::Mat& frame_) const {
  CV_Assert(frame_.type() == CV_8UC1 or frame_.type() == CV_8UC3);
  cv::drawChessboardCorners(frame_, cv::Size(_cols, _rows), _corners, true);
}
}  // namespace ca2lib