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

#include <gtest/gtest.h>

#include "ca2lib/targets.h"
#include "ca2lib/plane_extractor/plane_extractor_monocular.h"
#include "ca2lib/types.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/highgui.hpp>

void crvlMakeChessboardPattern(cv::Mat &outChessboard, size_t inRows,
                               size_t inCols, int inSquareSize = 100,
                               cv::Scalar color1 = cv::Scalar(0, 0, 0),
                               cv::Scalar color2 = cv::Scalar(255, 255, 255)) {
  using namespace cv;
  CV_Assert(inRows > 1 && inCols > 1);
  CV_Assert(inSquareSize > 0);

  size_t chessboardImgCols = inCols * inSquareSize;
  size_t chessboardImgRows = inRows * inSquareSize;
  outChessboard = Mat(chessboardImgRows + 200, chessboardImgCols + 200, CV_8UC3,
                      Scalar(255, 255, 255));

  for (size_t rows = 0, rowNum = 0; rows < chessboardImgRows;
       rows += inSquareSize, rowNum++) {
    for (size_t cols = 0, colNum = 0; cols < chessboardImgCols;
         cols += inSquareSize, colNum++) {
      Rect rec(cols + 100, rows + 100, inSquareSize, inSquareSize);
      if ((rowNum + colNum) % 2 == 0)
        rectangle(outChessboard, rec, color1, -1, 8);
      else
        rectangle(outChessboard, rec, color2, -1, 8);
    }
  }
}

TEST(ca2lib, ExtractorCheckerboard) {
  cv::Mat checkerboard;
  crvlMakeChessboardPattern(checkerboard, 8, 12);

  std::shared_ptr<ca2lib::TargetBase> target_ptr =
      std::make_shared<ca2lib::TargetCheckerboard>(7, 11, 0.1);

  cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
  K.at<float>(0, 0) = 1200.f;
  K.at<float>(1, 1) = 1200.f;
  K.at<float>(0, 2) = 600.f;
  K.at<float>(1, 2) = 600.f;

  float dist_coeffs_arr[] = {-0.11528735336289916, 0.21967978639309688,
                             -8.700204588710727e-05, 0.0015594039134848336,
                             0.090100677778518698};
  cv::Mat dist_coeffs = cv::Mat(5, 1, CV_32F, dist_coeffs_arr);

  ca2lib::PlaneExtractorMonocular extractor;
  extractor.setCameraParams(K, dist_coeffs);
  extractor.setTarget(target_ptr);
  extractor.setData(checkerboard);

  ASSERT_TRUE(extractor.process());

  cv::Mat detection = checkerboard.clone();
  target_ptr->drawDetection(detection);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
