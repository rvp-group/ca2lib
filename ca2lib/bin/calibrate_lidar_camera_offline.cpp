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

#include <ros/ros.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

#include <fstream>
#include <iostream>
#include <sstream>

#include "ca2lib/parse_command_line.h"
#include "ca2lib/solver/solver.h"
#include "ca2lib/types.h"

using namespace srrg2_core;

const char* whatdoes = "Calibrate LiDAR-RGB Camera pair from file.";

ca2lib::Solver solver;
Eigen::Isometry3f camera_T_lidar;

ca2lib::Measurements uploadMeasurements(std::string file_name) {
  ca2lib::Measurements measurements;

  std::ifstream file(file_name);

  if (file.is_open()) {
    std::string line;
    std::istringstream iss;
    int id = 0;
    while (std::getline(file, line)) {
      iss.clear();
      iss.str(line);

      ca2lib::Measurement measurement;
      ca2lib::Plane plane_lidar;
      ca2lib::Plane plane_camera;
      Eigen::Vector4f vec;

      std::getline(iss, line, ' ');
      vec[0] = std::stof(line);
      std::getline(iss, line, ' ');
      vec[1] = std::stof(line);
      std::getline(iss, line, ' ');
      vec[2] = std::stof(line);
      std::getline(iss, line, ',');
      vec[3] = std::stof(line);
      std::getline(iss, line, ' ');
      plane_lidar << vec;

      std::getline(iss, line, ' ');
      vec[0] = std::stof(line);
      std::getline(iss, line, ' ');
      vec[1] = std::stof(line);
      std::getline(iss, line, ' ');
      vec[2] = std::stof(line);
      std::getline(iss, line, '\n');
      vec[3] = std::stof(line);

      plane_camera << vec;

      measurement.from = plane_lidar;
      measurement.to = plane_camera;
      measurement.id = id++;
      measurements.push_back(measurement);

      spdlog::set_level(spdlog::level::debug);
      spdlog::debug("lidar plane: {}", plane_lidar);
      spdlog::debug("camera plane: {}", plane_camera);
    }

    file.close();
  } else {
    spdlog::error("Unable to open file: " + file_name);
  }

  return measurements;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "calibrate_lidar_camera_offline");

  ParseCommandLine parser(argv, &whatdoes);
  ArgumentString output_f(&parser, "o", "output",
                          "Output calibration file [ends for .yaml or .json]",
                          "lidar_in_camera.yaml");
  ArgumentString dump_planes(&parser, "p", "planes",
                             "Planes transformed after calibration",
                             "transformed_planes.txt");
  ArgumentString input_f(&parser, "i", "input",
                         "Input file containing the planes", "");

  ArgumentFloat inlier_threshold(
      &parser, "T", "threshold-inlier",
      "Threshold for a measurement to be considered inlier", 3.0f);

  ArgumentInt no_iterations(&parser, "n", "iterations",
                            "Number of solver iterations", 10);
  ArgumentFloat damping_factor(&parser, "d", "damping", "Damping factor", 10.f);
  ArgumentFloat huber_delta(&parser, "u", "huber", "Huber threshold", 0.1f);
  parser.parse();

  if (!input_f.isSet()) {
    spdlog::error("No measurements file set.\n");
    spdlog::error(parser.options());
    return -1;
  }

  spdlog::info("Reading planes from file: " + input_f.value());

  auto measurements = uploadMeasurements(input_f.value());

  if (measurements.size() > 3) {
    spdlog::info("Solving extrinsics camera_T_lidar");
    solver.measurements() = measurements;
    solver.dumping() = damping_factor.value();
    solver.iterations() = no_iterations.value();
    solver.inlierTh() = inlier_threshold.value();
    solver.setMEstimator(std::bind(ca2lib::huber, std::placeholders::_1,
                                   std::placeholders::_2, huber_delta.value()));
    solver.compute();

    spdlog::set_level(spdlog::level::debug);
    spdlog::debug(solver.stats());

    camera_T_lidar = solver.estimate();
    spdlog::debug("camera_T_lidar: \n{}\n", camera_T_lidar.matrix());
    spdlog::info("Solving terminated");
    if (solver.stats().back().status !=
        ca2lib::IterationStat::SolverStatus::Success)
      spdlog::error("Solving fails\n");

    ca2lib::CameraLidarExtrinsics camera_lidar_extrinsics(camera_T_lidar);
    camera_lidar_extrinsics.save(output_f.value());

    // save planes after calibration
    solver.dumpResult(dump_planes.value());

  } else {
    spdlog::error("No enough measurements.\n");
    return -1;
  }

  return 0;
}