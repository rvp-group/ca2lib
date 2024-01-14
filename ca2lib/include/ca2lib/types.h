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
#include <opencv2/core.hpp>
#include <vector>
#include <map>
#include <utility>
#include "geometry.h"

namespace ca2lib {

/**
 * @brief Internal representation of a Plane in Hesse normal form
 */
struct Plane {
 public:
  Plane() = default;
  ~Plane() = default;
  Plane(Eigen::Vector4f vec_) {
    _normal << vec_(0), vec_(1), vec_(2);
    _d = vec_(3);
  }

  inline void operator<<(Eigen::Vector4f vec_) {
    _normal << vec_(0), vec_(1), vec_(2);
    _d = vec_(3);
  }

  /**
   * @brief Plane's accessors
   */
  inline const Eigen::Vector3f& normal() const { return _normal; }
  inline Eigen::Vector3f& normal() { return _normal; }
  inline const float& d() const { return _d; }
  inline float& d() { return _d; }

  /**
   * @brief Set random value to plane's components
   */
  inline void setRandom() {
    Eigen::Vector4f random_vec = Eigen::Vector4f::Random();
    random_vec.head<3>().normalize();
    _normal = random_vec.head<3>();
    _d = random_vec(3);
  }

  inline bool operator==(const Plane& other) const {
    return (_normal == other.normal() && _d == other.d());
  }

  /**
   * @brief Transform a plane with an isometry T_
   * @param T_
   * @return return a Plane
   */
  inline Plane operator*(const Eigen::Isometry3f& T_) const {
    Plane plane;
    plane.normal() = T_.linear() * _normal;
    plane.d() = _d + plane.normal().transpose() * T_.translation();
    return plane;
  }

  /**
   * @brief Return the point on the plane closest to the origin
   * of the reference system
   * @return return the point as Eigen::Vector3f
   */
  inline Eigen::Vector3f pointInPlane() const {
    Eigen::Vector3f point = -1 * _normal * _d;
    return point;
  }

  /**
   * @brief Compute the error between two planes: this - plane_j
   * @return return the error as Eigen::Vector4f
   */
  inline Eigen::Vector4f operator-(const Plane& plane_j_) const {
    Eigen::Vector4f error = Eigen::Vector4f::Zero();
    error.head<3>() = this->normal() - plane_j_.normal();
    error(3) = plane_j_.normal().transpose() *
               (plane_j_.pointInPlane() - this->pointInPlane());
    // error(3) = this->d() - plane_j_.d();
    return error;
  }

  /**
   * @brief Compute the distance between a point and the plane
   * @param point_
   * @return return the distance as a float
   */
  inline float distancePointToPlane(const Eigen::Vector3f& point_) const {
    return (point_ - pointInPlane()).dot(_normal);
  }

  friend std::ostream& operator<<(std::ostream& out_, const Plane& p_);
  friend inline Plane operator*(const Eigen::Isometry3f& T_, const Plane& p_);

 protected:
  Eigen::Vector3f _normal = Eigen::Vector3f::Ones();
  float _d = 0;
};

inline std::ostream& operator<<(std::ostream& out_, const Plane& p_) {
  out_ << "normal: " << p_.normal() << ", " << p_.d() << "\n";
  return out_;
}

/**
 * @brief Transform a plane p_ with an isometry T_
 * @param T_
 * @return return a Plane
 */
inline Plane operator*(const Eigen::Isometry3f& T_, const Plane& p_) {
  Plane plane;
  plane.normal() = T_.linear() * p_.normal();
  plane.d() = p_.d() + plane.normal().transpose() * T_.translation();
  return plane;
}

/**
 * @brief Internal representation of a Multi channel point cloud
 * Assumption made is that the first three channels represents XYZ components of
 * the points
 *
 */
struct PointCloudXf {
 public:
  PointCloudXf() = default;
  PointCloudXf(PointCloudXf& other_) {
    height = other_.height;
    width = other_.width;
    points = other_.points;
    fields = other_.fields;
  }

  ~PointCloudXf() = default;

  /**
   * @brief Returns the number of fields that are contained in the cloud.
   *
   * @return unsigned int number of fields
   */
  unsigned int numFields() const { return fields.size(); }

  inline PointCloudXf& operator=(const PointCloudXf& other_) {
    height = other_.height;
    width = other_.width;
    points = other_.points;
    fields = other_.fields;
    return *this;
  }

  unsigned int height, width;
  std::vector<Eigen::VectorXf> points;
  /**
   * @brief fields associate each cloud channel to two values. The former is
   * the accessor index to retrive that field in a given point. The latter
   * represents the original data type (refer to
   * sensor_msgs::PointCloud2::PointField).
   */
  std::map<std::string, std::pair<unsigned int, uint8_t>> fields;
};

struct CameraIntrinsics {
  cv::Mat K;
  cv::Mat dist_coeffs;
  cv::Mat rvecs, tvecs;
  float reprojection_error;

  /**
   * @brief Save parameters in a [JSON|YAML] file
   *
   * @param f destination path
   *
   */
  void save(const std::string& f) const;
  /**
   * @brief Load parameters from a previously [JSON|YAML] file
   *
   * @param f source path
   * @return CameraIntrinsics
   */
  static CameraIntrinsics load(const std::string& f);
};

// Inverse LookUp Table
using InverseLut_t = std::vector<std::pair<bool, cv::Point2i>>;
}  // namespace ca2lib