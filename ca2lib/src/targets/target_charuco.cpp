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

#include "ca2lib/targets/target_charuco.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

namespace ca2lib {

CameraIntrinsics CalibrationDataCharuco::calibrateCamera(
    CameraModel model_type, DistortionModel dist_type) const {
  CameraIntrinsics res;
  switch (model_type) {
    case Pinhole: {
      switch (dist_type) {
        case None: {
          res.reprojection_error = cv::aruco::calibrateCameraCharuco(
              _corners, _corners_idx, _board, _image_size, res.K,
              res.dist_coeffs, res.rvecs, res.tvecs,
              cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_K1 |
                  cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3);
          break;
        }
        case RadTan: {
          res.reprojection_error = cv::aruco::calibrateCameraCharuco(
              _corners, _corners_idx, _board, _image_size, res.K,
              res.dist_coeffs, res.rvecs, res.tvecs);
          break;
        }
        default:
          throw std::runtime_error(
              "Distortion model is not supported by selected CameraModel");
      }
      break;
    }
    default:
      throw std::runtime_error("Unsupported CameraModel.");
  }
  return res;
}

void CalibrationDataCharuco::drawDetection(cv::Mat& frame) const {
  for (int i = 0; i < _corners.size(); ++i) {
    cv::aruco::drawDetectedCornersCharuco(frame, _corners[i], _corners_idx[i]);
  }
};

void CalibrationDataCharuco::reset() {
  _image_size = cv::Size(0, 0);
  _corners.clear();
  _corners_idx.clear();
};

TargetCharuco::TargetCharuco(unsigned int rows_, unsigned int cols_,
                             float len_square_, float len_marker_,
                             const Family_t family_) {
  cv::Ptr<cv::aruco::Dictionary> dict =
      cv::aruco::getPredefinedDictionary(family_);
  _board = cv::aruco::CharucoBoard::create(cols_, rows_, len_square_,
                                           len_marker_, dict);
  _parameters = cv::aruco::DetectorParameters::create();
}

/**
 * @brief Attempts target detection on the input image. In case target is
 * detected, stores detection parameters internally.
 *
 * @param frame_ Input image
 * @return true if target is detected
 * @return false if target is not detected
 */
bool TargetCharuco::detect(const cv::Mat& frame_) {
  CV_Assert(frame_.type() == CV_8UC1 or frame_.type() == CV_8UC3);
  cv::Mat frame_gray;
  if (frame_.type() == CV_8UC1)
    frame_gray = frame_;
  else
    cv::cvtColor(frame_, frame_gray, cv::COLOR_BGR2GRAY);

  _corners.clear();
  _corners_idx.clear();

  std::vector<std::vector<cv::Point2f>> marker_corners, rejected;
  std::vector<int> marker_ids;
  cv::aruco::detectMarkers(frame_gray, _board->dictionary, marker_corners,
                           marker_ids, _parameters, rejected);
  if (!marker_corners.size()) return false;

  cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, frame_gray,
                                       _board, _corners, _corners_idx);
  return true;
}

/**
 * @brief Attempts target detection on the input image and estimates relative
 * target position estimate. In case target is detected, stores detection and
 * relative pose parameters internally.
 *
 * @param frame_ Input image
 * @param K_ 3x3 Camera matrix (CV_32F or CV_64F)
 * @param dist_coeffs_ 1xM distortion parameters (CV_32 or CV_64F)
 * @return true if target is detected
 * @return false if target is not detected
 */
bool TargetCharuco::detectAndCompute(const cv::Mat& frame_, const cv::Mat& K_,
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
    ret = cv::aruco::estimatePoseCharucoBoard(undistorted_corners, _corners_idx,
                                              _board, identity, cv::Mat(),
                                              _rvec, _tvec);
  } else {
    ret = cv::aruco::estimatePoseCharucoBoard(_corners, _corners_idx, _board,
                                              K_, dist_coeffs_, _rvec, _tvec);
  }
  return ret;
}

/**
 * @brief Draw the detected target in frame
 *
 * @param frame_ Output image (CV_8UC1 or CV_8UC3)
 */
void TargetCharuco::drawDetection(cv::Mat& frame_) const {
  CV_Assert(frame_.type() == CV_8UC1 or frame_.type() == CV_8UC3);
  cv::aruco::drawDetectedCornersCharuco(frame_, _corners, _corners_idx);
}

/**
 * @brief Save the current detection inside the internal result representation
 *
 */
void TargetCharuco::saveDetection() {
  _storage._board = _board;
  _storage._image_size = _image_size;
  _storage._corners.push_back(_corners);
  _storage._corners_idx.push_back(_corners_idx);
}
}  // namespace ca2lib
