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
#include "ca2lib/types.h"
#include "plane_extractor_base.h"
#include "ca2lib/targets/target_base.h"

namespace ca2lib {

class PlaneExtractorMonocular : public PlaneExtractorBase {
 public:
  /**
   * @brief Set the Target object. This will be used to detect the target in the
   * input frame
   *
   * @param target_
   */
  inline void setTarget(const TargetBase::SharedPtr target_) {
    _target = target_;
  }

  /**
   * @brief Set the intrinsics of the camera. These will be used to estimate the
   * target pose in camera frame.
   *
   * @param K_
   * @param dist_coeffs_
   */
  inline void setCameraParams(const cv::Mat& K_, const cv::Mat& dist_coeffs_) {
    _K = K_;
    _dist_coeffs = dist_coeffs_;
  }

  /**
   * @brief Load the input frame.
   *
   * @param frame_ cv::Mat
   */
  inline void setData(const cv::Mat& frame_) { _frame = frame_; }

  /**
   * @brief Interface for PlaneExtractors. When called, the extractor process
   * input data to find the target plane.
   *
   * @return true plane was found
   * @return false otherwise
   */
  bool process() override;

  /**
   * @brief Draw 3D Axis attached to the detected plane.
   *
   * @param frame
   */
  void drawPlane(cv::Mat& frame);

 protected:
  TargetBase::SharedPtr _target;
  cv::Mat _K, _dist_coeffs;
  cv::Mat _frame;
};
}  // namespace ca2lib