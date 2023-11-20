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
#include "ca2lib/solver/solver.h"
#include "ca2lib/types.h"
#include "ca2lib/geometry.h"


TEST(ca2lib, SolverWithLessThan3Measurement) {
  ca2lib::Solver solver;
  ASSERT_FALSE(solver.compute());
}

TEST(ca2lib, SolverRandomMeasurement) {
  ca2lib::Measurements measurements;
  uint plane_num = 100;
  float eps = 1e-6;

  ca2lib::Vector6f pose;
  pose << 0.5,0.2,-0.3,0.2,-0.2,0.1;
  Eigen::Isometry3f T = ca2lib::v2t(pose);

  for(uint i=0; i < plane_num; ++i) {
    ca2lib::Plane p{};
    p.setRandom();

    ca2lib::Measurement m;
    m.id = i;
    m.from = p;
    m.to = T * p;
    
    measurements.push_back(m);
  }

  ca2lib::Solver solver;
  solver.dumping() = 1;
  // solver.estimate() = T;
  solver.iterations() = 10;
  solver.measurements() = measurements;
  solver.compute();

  std::cerr << solver.stats() << std::endl;

  Eigen::Isometry3f res = T.inverse() * solver.estimate();
  std::cerr << res.matrix() - Eigen::Matrix4f::Identity() << std::endl;

  std::cerr << "solver result: " << solver.estimate().matrix() << std::endl;
  
  Eigen::Matrix4f I = Eigen::Matrix4f::Identity();

  ASSERT_TRUE(I.isApprox(res.matrix(), eps));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

