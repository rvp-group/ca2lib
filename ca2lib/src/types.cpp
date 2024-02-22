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

#include "ca2lib/types.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace ca2lib {

void CameraIntrinsics::save(const std::string& f) const {
  namespace fs = std::filesystem;
  fs::path filename = f;
  if (filename.extension() == ".yaml") {
    YAML::Node conf;
    YAML::Node storage_K;
    storage_K.push_back(K.at<double>(0, 0));
    storage_K.push_back(K.at<double>(1, 1));
    storage_K.push_back(K.at<double>(0, 2));
    storage_K.push_back(K.at<double>(1, 2));
    YAML::Node storage_D;
    for (int i = 0; i < dist_coeffs.cols; ++i) {
      storage_D.push_back(dist_coeffs.at<double>(0, i));
    }
    conf["K"] = storage_K;
    conf["dist_coeffs"] = storage_D;
    YAML::Emitter emitter;
    emitter << conf;
    std::ofstream fout(f);
    fout << emitter.c_str();
    return;
  } else if (filename.extension() == ".json") {
    throw std::runtime_error("JSON configurations are currently not supported");
  } else {
    throw std::runtime_error("Format configuration is currently not supported");
  }
}

CameraIntrinsics CameraIntrinsics::load(const std::string& f) {
  namespace fs = std::filesystem;
  fs::path filename = f;
  CameraIntrinsics res;
  if (filename.extension() == ".yaml") {
    const auto config = YAML::LoadFile(f);
    res.K = cv::Mat::eye(3, 3, CV_64F);
    res.K.at<double>(0, 0) = config["K"][0].as<double>();
    res.K.at<double>(1, 1) = config["K"][1].as<double>();
    res.K.at<double>(0, 2) = config["K"][2].as<double>();
    res.K.at<double>(1, 2) = config["K"][3].as<double>();
    res.dist_coeffs = cv::Mat(1, config["dist_coeffs"].size(), CV_64F);
    for (int i = 0; i < res.dist_coeffs.cols; ++i) {
      res.dist_coeffs.at<double>(0, i) = config["dist_coeffs"][i].as<double>();
    }
  } else if (filename.extension() == ".json") {
    throw std::runtime_error("JSON configurations are currently not supported");
  } else {
    throw std::runtime_error("Format configuration is currently not supported");
  }
  return res;
}

void eigenMatrixToYamlNode(YAML::Node& node, Eigen::Isometry3f isometry) {
  for(int r=0; r < 3; ++r) {
    for (int c=0; c < 4; ++c) {
      node.push_back(isometry.matrix()(r,c));
    }
  }
  return;
}

void eigenMatrixRodriguesToYamlNode(YAML::Node& node, Eigen::Isometry3f isometry) {
  cv::Mat R, rvec;
  cv::eigen2cv((Eigen::Matrix3f)isometry.linear(), R);
  cv::Rodrigues(R, rvec);
  for(int r=0; r < 3; ++r) {
    node.push_back(rvec.at<double>(0,r));
  }
  for (int r = 0; r<3; ++r) {
    node.push_back(isometry.translation()(r));
  }
  return;
}

void CameraLidarExtrinsics::save(const std::string& f) const {
  namespace fs = std::filesystem;
  fs::path filename = f;
  if (filename.extension() == ".yaml") {
    YAML::Node conf;
    YAML::Node storage_l_in_c, storage_c_in_l;
    eigenMatrixToYamlNode(storage_l_in_c, lidar_in_camera);
    eigenMatrixToYamlNode(storage_c_in_l, camera_in_lidar);

    conf["lidar_in_camera"] = storage_l_in_c;
    conf["camera_in_lidar"] = storage_c_in_l;

    YAML::Node storage_l_in_c_r, storage_c_in_l_r;

    eigenMatrixRodriguesToYamlNode(storage_l_in_c_r, lidar_in_camera);
    eigenMatrixRodriguesToYamlNode(storage_c_in_l_r, camera_in_lidar);

    conf["lidar_in_camera_rodrigues"] = storage_l_in_c_r;
    conf["camera_in_lidar_rodrigues"] = storage_c_in_l_r;

    YAML::Emitter emitter;
    emitter << conf;
    std::ofstream fout(f);
    fout << emitter.c_str();
    return;
  } else if (filename.extension() == ".json") {
    throw std::runtime_error("JSON configurations are currently not supported");
  } else {
    throw std::runtime_error("Format configuration is currently not supported");
  }
  return;
}

CameraLidarExtrinsics CameraLidarExtrinsics::load(const std::string& f) {
  namespace fs = std::filesystem;
  fs::path filename = f;
  CameraLidarExtrinsics res;
  if (filename.extension() == ".yaml") {
    const auto config = YAML::LoadFile(f);

    for (int r = 0; r < 3; r++) {
      for(int c = 0; c < 4; c++) {
        res.lidar_in_camera.matrix()(r,c) = config["lidar_in_camera"][r*4+c].as<double>();
      }
    }

    res.camera_in_lidar = res.lidar_in_camera.inverse();

  } else if (filename.extension() == ".json") {
    throw std::runtime_error("JSON configurations are currently not supported");
  } else {
    throw std::runtime_error("Format configuration is currently not supported");
  }
  return res;
}

}  // namespace ca2lib