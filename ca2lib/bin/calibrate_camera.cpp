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

#include <spdlog/spdlog.h>

#include <filesystem>
#include <opencv2/highgui.hpp>

#include "ca2lib/parse_command_line.h"
#include "ca2lib/targets.h"

using namespace srrg2_core;

const char* whatdoes =
    "Calibrate a single camera given a set of input images and the "
    "corresponding calibration target";

int main(int argc, char** argv) {
  ParseCommandLine parser(argv, &whatdoes);

  ArgumentString input_dir(
      &parser, "i", "input-dir",
      "Directory containing calibration images for the camera", "");
  ArgumentString camera_model(&parser, "m", "model", "Camera model to use",
                              "pinhole");
  ArgumentString dist_model(&parser, "d", "distortion",
                            "Distortion model to use", "radtan");
  ArgumentString output_f(&parser, "o", "output",
                          "Output calibration file [ends for .yaml or .json]",
                          "calibration_result.yaml");
  ArgumentString target_f(&parser, "t", "target",
                          "YAML file containing target metadata", "");

  parser.parse();

  if (!input_dir.isSet() or !target_f.isSet()) {
    std::cerr << "No input folder or target specified.\n"
              << parser.options() << std::endl;
    return -1;
  }
  ca2lib::TargetBase::SharedPtr target =
      ca2lib::TargetBase::fromFile(target_f.value());

  auto& detections = target->getData();

  std::filesystem::path dir_images = input_dir.value();

  std::vector<cv::Mat> images;

  cv::Mat frame_viz;

  for (const auto f : std::filesystem::directory_iterator{dir_images}) {
    spdlog::info("Reading " + f.path().string());
    cv::Mat image = cv::imread(f.path(), cv::ImreadModes::IMREAD_UNCHANGED);
    if (frame_viz.empty()) frame_viz = cv::Mat(image.size(), CV_8UC3);
    frame_viz = cv::Vec3b::all(255);

    images.push_back(image);
    target->detect(image);
    target->saveDetection();
    detections.drawDetection(frame_viz);
    cv::imshow("Detections", frame_viz);
    cv::waitKey(10);
  }

  spdlog::info("Running calibration");
  const auto calibration_results = detections.calibrateCamera(
      ca2lib::CalibrationData::CameraModel::Pinhole,
      ca2lib::CalibrationData::DistortionModel::RadTan);
  spdlog::info("Calibration results:");
  std::cerr << "reprojection error = " << calibration_results.reprojection_error
            << std::endl;
  std::cerr << "K = " << calibration_results.K << std::endl;

  std::cerr << "distortion coefficients = " << calibration_results.dist_coeffs
            << std::endl;
  spdlog::info("Saving results to " + output_f.value());
  calibration_results.save(output_f.value());
  return 0;
}