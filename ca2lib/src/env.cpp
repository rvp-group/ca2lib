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

#include "ca2lib/env.h"

#include <iostream>
#include <map>
#include <string>

extern char** environ;

namespace srrg2_core {
using namespace std;

std::map<std::string, std::string> _env_map;

void initEnvMap() {
  // cerr << "InitEnvMap" << endl;
  if (_env_map.size()) {
    // cerr << "skip" << endl;
    return;
  }
  _env_map.clear();
  char** e = environ;
  while (*e) {
    std::string key_value(*e);
    ++e;
    auto pos = key_value.find_first_of('=');
    if (pos == std::string::npos) continue;
    std::string key = key_value.substr(0, pos);
    std::string value = key_value.substr(pos + 1);
    _env_map.insert(std::make_pair(key, value));
  }
  // cerr << "env_map: addr " << &_env_map << ", size " << _env_map.size() <<
  // endl;
}

const std::map<std::string, std::string>& envMap() { return _env_map; }

void envMapAddCmdArgs(const std::vector<std::string>& args) {
  for (size_t i = 0; i < args.size(); ++i) {
    std::string key = std::string("ARG_") + std::to_string(i);
    _env_map.insert(std::make_pair(key, args[i]));
  }
  // cerr << "env_map: addr " << &_env_map << ", size " << _env_map.size() <<
  // endl;
}

void replaceEnvTags(std::string& str) {
  using namespace std;
  string::iterator begin_tag = str.end();
  for (string::iterator it = str.begin(); it != str.end(); it++) {
    if (*it == '<') {
      begin_tag = it;
      continue;
    }
    if (*it == '>' && begin_tag < it) {
      std::string key =
          str.substr(begin_tag + 1 - str.begin(), it - begin_tag - 1);
      const auto& k_it = _env_map.find(key);
      std::string replacement("");
      if (k_it != _env_map.end()) {
        replacement = k_it->second;
      } else {
        cerr << "no  tag <" << key << "> in env" << endl;
      }
      size_t newpos = begin_tag - str.begin() + replacement.length() - 1;
      str.replace(begin_tag, it + 1, replacement);
      it = str.begin() + newpos;
    }
  }
}

}  // namespace srrg2_core