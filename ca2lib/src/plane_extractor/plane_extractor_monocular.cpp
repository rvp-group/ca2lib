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

#include "ca2lib/plane_extractor/plane_extractor_monocular.h"
#include <opencv2/calib3d.hpp>
#include <iostream>

namespace ca2lib {

/**
 * @brief Interface for PlaneExtractors. When called, the extractor process
 * input data to find the target plane.
 *
 * @return true plane was found
 * @return false otherwise
 */
bool PlaneExtractorMonocular::process() {
  if (!_target) throw std::runtime_error("No target set.");

  bool target_found = _target->detectAndCompute(_frame, _K, _dist_coeffs);
  if (!target_found) return false;

  cv::Vec3f normal_in_target = cv::Vec3f(0.0f, 0.0f, 1.0f);

  cv::Mat R;
  cv::Rodrigues(_target->rvec(), R);
  R.convertTo(R, CV_32F);
  const cv::Mat normal_in_camera = R * cv::Mat(normal_in_target);
  cv::Mat target_in_camera_translation = _target->tvec();
  target_in_camera_translation.convertTo(target_in_camera_translation, CV_32F);

  float d = normal_in_camera.dot(target_in_camera_translation);

  _plane.normal() << normal_in_camera.at<float>(0), normal_in_camera.at<float>(1), normal_in_camera.at<float>(2);
  _plane.d() = d;

  return true;
}

/**
 * @brief Draw 3D Axis attached to the detected plane.
 *
 * @param frame
 */
void PlaneExtractorMonocular::drawPlane(cv::Mat& frame_) {
  // TODO: Check that this works also for fisheye model

  cv::drawFrameAxes(frame_, _K, _dist_coeffs, _target->rvec(), _target->tvec(),
                    0.1);
}

}  // namespace ca2lib