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
#include "ca2lib/geometry.h"

#define EPS 1e-4


TEST(ca2lib, TestskewMatrix) {
  Eigen::Vector3f vec;
  vec << 1,2,3;
  Eigen::Matrix3f skew = ca2lib::skew(vec);
  Eigen::Matrix3f skew_t = -1 * skew.transpose();
  ASSERT_TRUE(skew.isApprox(skew_t, EPS));
}


TEST(ca2lib, TestRx) {
  Eigen::Vector3f vec;
  vec << 1,2,3;
  
  Eigen::Matrix3f rx = ca2lib::Rx(0.3f);

  Eigen::Vector3f rotated_vec;
  rotated_vec << 1.0000,1.0241,3.4570;

  ASSERT_TRUE(rotated_vec.isApprox(rx * vec, EPS));
}

TEST(ca2lib, TestRy) {
  Eigen::Vector3f vec;
  vec << 1,2,3;
  
  Eigen::Matrix3f ry = ca2lib::Ry(0.3f);

  Eigen::Vector3f rotated_vec;
  rotated_vec << 1.8419,2.0000,2.5705;

  ASSERT_TRUE(rotated_vec.isApprox(ry * vec, EPS));
}

TEST(ca2lib, TestRz) {
  Eigen::Vector3f vec;
  vec << 1,2,3;
  
  Eigen::Matrix3f rz = ca2lib::Rz(0.3f);

  Eigen::Vector3f rotated_vec;
  rotated_vec << 0.36430,2.20619,3.00000;

  ASSERT_TRUE(rotated_vec.isApprox(rz * vec, EPS));
}

TEST(ca2lib, Testv2tIdentity) {
  ca2lib::Vector6f pose;
  pose <<0,0,0,0,0,0;

  Eigen::Isometry3f T = ca2lib::v2t(pose);

  Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
  Eigen::Vector3f t;
  t << 0,0,0;

  ASSERT_TRUE(I.isApprox(T.linear(), EPS) && t.isApprox(T.translation(), EPS));
}

TEST(ca2lib, Testangle2RIdentity) {
  Eigen::Vector3f eul;
  eul << 0,0,0;
  Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
  
  Eigen::Matrix3f R = ca2lib::angle2R(eul);

  ASSERT_TRUE(I.isApprox(R, EPS));
}

TEST(ca2lib, Testangle2R) {
  Eigen::Vector3f vec;
  vec << 1,2,3;

  Eigen::Vector3f eul;
  eul << 0.2,0.3,0.4;
  
  Eigen::Matrix3f R = ca2lib::angle2R(eul);

  Eigen::Vector3f rotated_vec;
  rotated_vec << 1.0224,1.6260,3.2110;

  ASSERT_TRUE(rotated_vec.isApprox(R * vec, EPS));
}

TEST(ca2lib, Testv2t) {
  Eigen::Vector3f vec;
  vec << 1,2,3;

  ca2lib::Vector6f pose;
  pose <<1,2,3,0.2,0.3,0.4;

  Eigen::Isometry3f T = ca2lib::v2t(pose);

  Eigen::Vector3f transfor_vec;
  transfor_vec << 2.0224,3.6260,6.2110;

  ASSERT_TRUE(transfor_vec.isApprox(T * vec, EPS));
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

