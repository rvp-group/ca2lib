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

#include <gtest/gtest.h>
#include "ca2lib/solver/solver.h"
#include "ca2lib/types.h"
#include "ca2lib/geometry.h"

TEST(ca2lib, SolverNotEnoughMeasurements) {
  ca2lib::Solver solver;
  ASSERT_FALSE(solver.compute());
  ASSERT_EQ(solver.stats().back().status,
            ca2lib::IterationStat::SolverStatus::NotEnoughMeasurements);
}

TEST(ca2lib, SolverRandomMeasurement) {
  ca2lib::Measurements measurements;
  uint plane_num = 10;
  float eps = 1e-6;

  ca2lib::Vector6f pose;
  pose << 0.5, 0.2, -0.3, 0.2, -0.2, 0.1;
  Eigen::Isometry3f T = ca2lib::v2t(pose);

  for (uint i = 0; i < plane_num; ++i) {
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

  std::cerr << "solver result: " << std::endl
            << solver.estimate().matrix() << std::endl;

  std::cerr << "solution covariance: " << std::endl
            << solver.informationMatrix().inverse() << std::endl;

  Eigen::Matrix4f I = Eigen::Matrix4f::Identity();

  ASSERT_TRUE(I.isApprox(res.matrix(), eps));
  ASSERT_EQ(solver.stats().back().status,
            ca2lib::IterationStat::SolverStatus::Success);
}

TEST(ca2lib, SolverRealCal) {
  ca2lib::Measurements measurements;

  ca2lib::Measurement m0;
  m0.id = 0;
  m0.from << Eigen::Vector4f(-0.973557, 0.225927, 0.0338211, 2.30088);
  m0.to << Eigen::Vector4f(2.92599e-05, 0.000163091, -1, 1.99951);
  measurements.push_back(m0);

  ca2lib::Measurement m1;
  m1.id = 1;
  m1.from << Eigen::Vector4f(-0.549998, 0.813407, -0.189396, 1.7672);
  m1.to << Eigen::Vector4f(-0.706345, 0.00133016, -0.707867, 1.41308);
  measurements.push_back(m1);

  ca2lib::Measurement m2;
  m2.id = 2;
  m2.from << Eigen::Vector4f(-0.618524, 0.359442, 0.698734, 1.83563);
  m2.to << Eigen::Vector4f(0.000997042, -0.705697, -0.708514, 1.76834);
  measurements.push_back(m2);

  ca2lib::Measurement m3;
  m3.id = 3;
  m3.from << Eigen::Vector4f(-0.578783, 0.747195, 0.326664, 1.45452);
  m3.to << Eigen::Vector4f(-0.479101, -0.473493, -0.739098, 1.2336);
  measurements.push_back(m3);

  ca2lib::Measurement m4;
  m4.id = 4;
  m4.from << Eigen::Vector4f(-0.89066, 0.150572, -0.429014, 1.82543);
  m4.to << Eigen::Vector4f(-0.0936365, 0.45301, -0.886575, 1.45406);
  measurements.push_back(m4);

  std::cerr << measurements.size() << std::endl;

  ca2lib::Solver solver;
  solver.dumping() = 1;
  // solver.estimate() = T;
  solver.iterations() = 10;
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

  ASSERT_EQ(solver.stats().back().status,
            ca2lib::IterationStat::SolverStatus::Success);
}

TEST(ca2lib, SolverNotWellConstrained) {
  ca2lib::Measurements measurements;
  ca2lib::Plane p;
  p.normal() << 1, 0, 0;
  p.d() = 0;
  int plane_num = 6;

  ca2lib::Vector6f pose;
  pose << 0.5, 0.2, -0.3, 0.2, -0.2, 0.1;
  Eigen::Isometry3f T = ca2lib::v2t(pose);

  for (uint i = 0; i < plane_num; ++i) {
    ca2lib::Measurement m;
    m.id = i;
    m.from = p;
    m.to = T * p;

    measurements.push_back(m);
  }

  ca2lib::Solver solver;
  solver.dumping() = 1;
  solver.iterations() = 10;
  solver.measurements() = measurements;
  solver.compute();

  std::cerr << solver.stats() << std::endl;

  ASSERT_FALSE(solver.compute());
  ASSERT_EQ(solver.stats().back().status,
            ca2lib::IterationStat::SolverStatus::NotWellConstrained);
}

TEST(ca2lib, SolverUnBalance) {
  ca2lib::Measurements measurements;
  uint plane_num = 30;
  float eps = 1e-6;

  ca2lib::Vector6f pose;
  pose << 0.5, 0.2, -0.3, 0.2, -0.2, 0.1;
  Eigen::Isometry3f T = ca2lib::v2t(pose);

  ca2lib::Plane p{};
  for (uint i = 0; i < plane_num; ++i) {
    p.setRandom();

    ca2lib::Measurement m;
    m.id = i;
    m.from = p;
    m.to = T * p;

    measurements.push_back(m);
  }

  for (uint i = 1; i <= plane_num; ++i) {
    ca2lib::Measurement m;
    m.id = i + plane_num;
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

  std::cerr << "solver result: " << std::endl
            << solver.estimate().matrix() << std::endl;

  std::cerr << "solution covariance: " << std::endl
            << solver.informationMatrix().inverse() << std::endl;

  Eigen::Matrix4f I = Eigen::Matrix4f::Identity();

  ASSERT_TRUE(I.isApprox(res.matrix(), eps));
  ASSERT_EQ(solver.stats().back().status,
            ca2lib::IterationStat::SolverStatus::UnBalance);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
