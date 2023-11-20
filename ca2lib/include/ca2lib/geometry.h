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

#pragma once

#include <Eigen/Dense>

namespace ca2lib {

  using Matrix6f = Eigen::Matrix<float, 6, 6>;
  using Vector6f = Eigen::Matrix<float, 6, 1>;

  /**
   * @brief Compute the skew matrix 3x3 of a vector 3x1
   * @param vec
   * @return return the skew matrix Eigen::Matrix3f  
   */
  inline Eigen::Matrix3f skew(const Eigen::Vector3f& vec) {
    Eigen::Matrix3f skew = Eigen::Matrix3f::Zero();
    skew(0,1) = -vec.z();
    skew(0,2) = vec.y();
    skew(1,0) = vec.z();
    skew(1,2) = -vec.x();
    skew(2,0) = -vec.y();
    skew(2,1) = vec.x();
    return skew;
  }

  /**
   * @brief Compute the rotation matrix 3x3 around x-axis of angle rot_x_
   * @param rot_x_
   * @return return the rotation matrix Eigen::Matrix3f  
   */
  inline Eigen::Matrix3f Rx(const float& rot_x_) {
    float c = cos(rot_x_);
    float s = sin(rot_x_);
    Eigen::Matrix3f R;
    R << 1,  0,   0,
         0,  c,  -s,
         0,  s,   c;
    return R;
  }

  /**
   * @brief Compute the rotation matrix 3x3 around y-axis of angle rot_y_
   * @param rot_y_
   * @return return the rotation matrix Eigen::Matrix3f  
   */
  inline Eigen::Matrix3f Ry(const float& rot_y_) {
    float c = cos(rot_y_);
    float s = sin(rot_y_);
    Eigen::Matrix3f R;
    R << c,  0,  s,
         0,  1,  0,
        -s,  0,  c;
    return R;
  }

  /**
   * @brief Compute the rotation matrix 3x3 around z-axis of angle rot_z_
   * @param rot_z_
   * @return return the rotation matrix Eigen::Matrix3f  
   */
  inline Eigen::Matrix3f Rz(const float& rot_z_) {
    float c = cos(rot_z_);
    float s = sin(rot_z_);
    Eigen::Matrix3f R;
    R << c,  -s,  0,
         s,   c,  0,
         0,   0,  1;
    return R;
  }

  /**
   * @brief Compute the rotation matrix 3x3 from the Euler angles RPY
   * @param a_
   * @return return the rotation matrix Eigen::Matrix3f  
   */
  inline Eigen::Matrix3f angle2R(const Eigen::Vector3f& a_) {
    Eigen::Matrix3f R;
    R = Rx(a_(0))*Ry(a_(1))*Rz(a_(2));
    return R;
  }

  /**
   * @brief Compute the Isometry (R,t) from vec_ compose by 
   * translation and euler angles
   * @param vec_
   * @return return the Isometry 
   */
  inline Eigen::Isometry3f v2t(const Vector6f& vec_) {
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    T.translation()       = vec_.head<3>();
    T.linear()            = angle2R(vec_.tail<3>());
    return T;
  }
}  // namespace ca2lib