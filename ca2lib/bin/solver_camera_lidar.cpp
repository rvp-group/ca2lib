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
#include "ca2lib/parse_command_line.h"
#include "ca2lib/solver/solver.h"
#include <fstream>
#include <sstream>
#include <iostream>

using namespace srrg2_core;

const char* whatdoes = "Calibrate LiDAR-RGB Camera pair.";

using Planes = std::vector<ca2lib::Plane>;

Planes loadMeasurementsFromFile(const std::string& fin_) {
  Planes planes;

  std::ifstream file(fin_);
  while (!file.eof()) {
    std::string line;
    std::getline(file, line);
    std::istringstream is(line);
    Eigen::Vector4f vec;
    is >> vec(0) >> vec(1) >> vec(2) >> vec(3);
    ca2lib::Plane p;
    p << vec;
    planes.push_back(p);
  }

  file.close();
  return planes;
}

int main(int argc, char** argv) {
  ParseCommandLine parser(argv, &whatdoes);
  srrg2_core::ArgumentString camera_planes_file(
      &parser, "c", "camera-file-name",
      "file containing the plane extracted from camera", "");
  srrg2_core::ArgumentString lidar_planes_file(
      &parser, "l", "lidar-file-name",
      "file containing the plane extracted from lidar", "");

  parser.parse();

  if (!camera_planes_file.isSet()) {
    std::cout << "No camera planes passed" << std::endl;
    std::cout << parser.options() << std::endl;
    exit(-1);
  }

  if (!lidar_planes_file.isSet()) {
    std::cout << "No lidar planes passed" << std::endl;
    std::cout << parser.options() << std::endl;
    exit(-1);
  }

  Planes camera_planes = loadMeasurementsFromFile(camera_planes_file.value());
  Planes lidar_planes = loadMeasurementsFromFile(lidar_planes_file.value());

  if (camera_planes.size() != lidar_planes.size()) {
    std::cout << "Not equal planes for camera and lidar" << std::endl;
    std::cout << parser.options() << std::endl;
    exit(-1);
  }

  ca2lib::Measurements measurements;

  for (size_t index = 0; index < camera_planes.size(); ++index) {
    ca2lib::Measurement m;
    m.id = index;
    m.to = lidar_planes[index];
    m.from = camera_planes[index];
    measurements.push_back(m);
  }

  ca2lib::Solver solver;
  solver.dumping() = 1;
  // solver.estimate() = T;
  solver.iterations() = 100;
  solver.measurements() = measurements;
  solver.setMEstimator(std::bind(ca2lib::huber, std::placeholders::_1,
                                 std::placeholders::_2, 5.f));
  solver.inlierTh() = 10.f;
  solver.compute();

  std::cerr << solver.stats().back().measurement_stats;

  std::cerr << solver.stats() << std::endl;

  std::cerr << "solver result: " << std::endl
            << solver.estimate().matrix() << std::endl;

  std::cerr << "solution covariance: " << std::endl
            << solver.informationMatrix().inverse() << std::endl;

  return 0;
}