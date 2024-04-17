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
#include "target_base.h"

namespace ca2lib {

class CalibrationDataCheckerboard : public CalibrationData {
 public:
  CameraIntrinsics calibrateCamera(CameraModel, DistortionModel) const override;
  void drawDetection(cv::Mat&) const override;
  void reset() override;

  cv::Size _image_size;
  cv::Size _pattern_size;
  std::vector<std::vector<cv::Point2f>> _corners;
  std::vector<std::vector<cv::Point3f>> _grid_points;
};

/**
 * @brief Checkerboard calibration target.
 *
 */
class TargetCheckerboard : public TargetBase {
 public:
  /**
   * @brief Construct a new Target Checkerboard object
   *
   * @param rows_ number of internal rows corners
   * @param cols_ number of internal cols corners
   * @param grid_size_ distance in meters between corners
   */
  TargetCheckerboard(unsigned int rows_, unsigned int cols_, float grid_size_)
      : _rows(rows_), _cols(cols_), _grid_size(grid_size_) {
    for (unsigned int r = 0; r < _rows; ++r)
      for (unsigned int c = 0; c < _cols; ++c)
        _grid_points.push_back(
            cv::Point3f(_grid_size * r, _grid_size * c, 0.0f));
  }

  /**
   * @brief Attempts target detection on the input image. In case target is
   * detected, stores detection parameters internally.
   *
   * @param frame_ Input image
   * @return true if target is detected
   * @return false if target is not detected
   */
  bool detect(const cv::Mat& frame_) override;

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
  bool detectAndCompute(const cv::Mat& frame_, const cv::Mat& K_,
                        const cv::Mat& dist_coeffs_) override;

  /**
   * @brief Draw the detected target in frame
   *
   * @param frame_ Output image (CV_8UC1 or CV_8UC3)
   */
  void drawDetection(cv::Mat& frame_) const override;

  /**
   * @brief Save the current detection inside the internal result representation
   *
   */
  void saveDetection() override;

  /**
   * @brief Returns a constant reference to the internal result storage
   *
   * @return const CalibrationData&
   */
  inline const CalibrationData& getData() const { return _storage; };

 protected:
  unsigned int _rows, _cols;
  float _grid_size;

  std::vector<cv::Point2f> _corners;
  std::vector<cv::Point3f> _grid_points;
  cv::Size _image_size;

  CalibrationDataCheckerboard _storage;
};
}  // namespace ca2lib