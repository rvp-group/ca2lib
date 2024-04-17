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

#include "ca2lib/targets.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <spdlog/spdlog.h>

namespace ca2lib {

TargetBase::SharedPtr TargetBase::fromFile(const std::string& filename_str) {
  namespace fs = std::filesystem;
  TargetBase::SharedPtr ret = nullptr;

  fs::path filename = filename_str;
  if (filename.extension() == ".yaml") {
    const auto conf = YAML::LoadFile(filename_str);
    if (conf["type"].as<std::string>() == "chessboard" or
        conf["type"].as<std::string>() == "checkerboard") {
      ret = std::make_shared<TargetCheckerboard>(
          conf["rows"].as<unsigned int>(), conf["cols"].as<unsigned int>(),
          conf["grid_size"].as<float>());
      return ret;
    } else if (conf["type"].as<std::string>() == "charuco") {
      // TODO: Handle convertion from string to TargetCharuco::Family_t
      spdlog::warn("Instantiating a Charuco target with family=DICT_5x5_50");
      ret = std::make_shared<TargetCharuco>(
          conf["rows"].as<unsigned int>(), conf["cols"].as<unsigned int>(),
          conf["len_square"].as<float>(), conf["len_marker"].as<float>(),
          ca2lib::TargetCharuco::Family_t::DICT_5X5_50);
      return ret;
    }
  } else {
    throw std::runtime_error(
        "Only YAML configurations are currently supported");
  }
}
}  // namespace ca2lib