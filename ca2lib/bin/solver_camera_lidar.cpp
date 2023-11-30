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
  while (! file.eof())
  {
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
  srrg2_core::ArgumentString camera_planes_file(&parser,
      "c",
      "camera-file-name",
      "file containing the plane extracted from camera", "");
  srrg2_core::ArgumentString lidar_planes_file(&parser,
      "l",
      "lidar-file-name",
      "file containing the plane extracted from lidar", "");

  parser.parse();

  if(!camera_planes_file.isSet()) {
    std::cout << "No camera planes passed" << std::endl;
    std::cout << parser.options() << std::endl;
    exit(-1);
  }

  if(!lidar_planes_file.isSet()) {
    std::cout << "No lidar planes passed" << std::endl;
    std::cout << parser.options() << std::endl;
    exit(-1);
  }

  Planes camera_planes = loadMeasurementsFromFile(camera_planes_file.value());
  Planes lidar_planes = loadMeasurementsFromFile(lidar_planes_file.value());

  if(camera_planes.size() != lidar_planes.size()) {
    std::cout << "Not equal planes for camera and lidar" << std::endl;
    std::cout << parser.options() << std::endl;
    exit(-1);
  }

  ca2lib::Measurements measurements;

  for (size_t index = 0; index < camera_planes.size(); ++index){
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
  solver.setMEstimator(std::bind(ca2lib::huber, std::placeholders::_1, std::placeholders::_2, 5.f));
  solver.inlierTh() = 10.f;
  solver.compute();

  std::cerr << solver.stats().back().measurement_stats;

  std::cerr << solver.stats() << std::endl;

  std::cerr << "solver result: " << std::endl << solver.estimate().matrix() << std::endl;

  std::cerr << "solution covariance: " << std::endl << solver.informationMatrix().inverse() << std::endl;
 

  return 0;
}