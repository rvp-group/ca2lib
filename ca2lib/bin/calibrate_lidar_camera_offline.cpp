#include <iostream>
#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

#include "ca2lib/parse_command_line.h"
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
      measurement.to   = plane_camera;
      measurement.id = id++;
      measurements.push_back(measurement);

      spdlog::set_level(spdlog::level::debug);
      spdlog::debug("lidar plane: {}", plane_lidar);
      spdlog::debug("camera plane: {}", plane_camera);

    }

    file.close();
  } else {
    spdlog::error("Unable to open file: "+file_name);
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
    // solver.estimate().matrix() << 0, -1, 0, -0.3, 0, 0, -1, -0.53, 1, 0, 0, -0.01, 0, 0, 0, 1;
    solver.dumping() = 10;
    solver.iterations() = 10;
    solver.inlierTh() = 3.f;
    solver.setMEstimator(std::bind(ca2lib::huber, std::placeholders::_1, std::placeholders::_2, 0.1f));
    solver.compute();

    spdlog::set_level(spdlog::level::debug);
    spdlog::debug(solver.stats());

    camera_T_lidar = solver.estimate();
    spdlog::debug("camera_T_lidar: \n{}\n", camera_T_lidar.matrix());
    spdlog::info("Solving terminated");
    if(solver.stats().back().status != ca2lib::IterationStat::SolverStatus::Success)
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