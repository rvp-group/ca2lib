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
#include <vector>
#include "ca2lib/types.h"
#include <functional>

namespace ca2lib {

/**
 * @brief Representation of a single measurement: same plane measured from 
 * two different reference system 
 * to is the plane that I will obtain if I transform the plane from with the estimate.
 * to = estimate_T * from
 */
struct Measurement {
  Plane from;
  Plane to;
  int id;
  Measurement() = default;
  Measurement(const Plane& from_, const Plane& to_, int id_) :
    from(from_),
    to(to_),
    id(id_) {};
};

inline std::ostream& operator<<(std::ostream& out_, const Measurement& m_)
{
    out_ << "FROM: " << m_.from << "\n";
    out_ << "TO: " << m_.to << "\n";
    return out_;
}

using Measurements = std::vector<Measurement>;

/**
 * @brief Statistics for a single measurement
 */
struct MeasurementStat {
  enum class Status {Inlier, Outlier, Kernelized};
  Status status;
  float chi;
  MeasurementStat() = default;
};

using MeasurementStats = std::unordered_map<int, MeasurementStat>;

/**
 * @brief Statistics of one solver iteration
 */
struct IterationStat {
  MeasurementStats measurement_stats;
  int iteration_number;
  int num_inliers = 0;
  float chi_inliers = 0.f;
  int num_outliers = 0;
  float chi_outliers = 0.f;
  enum class SolverStatus {Success, NotEnoughMeasurements, NotWellConstrained};
  SolverStatus status;
  // IterationStat() = default;
};

using SolverStat = std::vector<IterationStat>;

std::ostream& operator<<(std::ostream& os, const IterationStat& istat_);
std::ostream& operator<<(std::ostream& os, const SolverStat& stats_);


/**
 * @brief Solver types
 */
using SensorOffset = Eigen::Isometry3f;
using InformationMatrix = Matrix6f;
using ErrorType = Eigen::Vector4f;
using JacobianType = Eigen::Matrix<float, 4, 6>;
using MEstimatorType = std::function<MeasurementStat::Status(const float&, float&)>;

/**
 * @brief Default M-estimator https://en.wikipedia.org/wiki/Huber_loss
 */
MeasurementStat::Status huber(const float& chi_, float& weight_, float error_threshold_);

/**
 * @brief Basic solver to resolve plane fitting for two sensors offset estimation
 * _measurements: set of measurements
 * _estimate: estimate of the offset
 * _omega: information matrix of the offset
 * _H, _b: matricies for linear solving
 * _stats: solver stats about each iteration
 * _iteration: number of gauss-newton iterations to perform
 * _dumping: dumping factor
 * _inlier_th: inlier threshold
 * mEstimator: robustifier function
 */
class Solver {
 public:

  Solver()  = default;
  ~Solver() = default;

  /**
   * @brief Once the measurements have been set (at least 3), 
   * it computes the sensor offeset _estimate
   * @return return true if success, false
   */
  bool compute();

  /**
   * @brief Solver's accessors
   */
  inline const InformationMatrix& informationMatrix() const {
    return _omega;
  }
  inline const SolverStat& stats() const {
    return _stats;
  }
  inline const SensorOffset& estimate() const {
    return _estimate;
  }
  inline SensorOffset& estimate() {
    return _estimate;
  }
  inline const int& iterations() const {
    return _iterations;
  }
  inline int& iterations() {
    return _iterations;
  }
  inline const float& dumping() const {
    return _dumping;
  }
  inline float& dumping() {
    return _dumping;
  }
  inline const Measurements& measurements() const {
    return _measurements;
  }
  inline Measurements& measurements() {
    return _measurements;
  }
  inline void setMEstimator(MEstimatorType mEstimator_) {
    mEstimator = mEstimator_;
    return;
  }
  inline const float& inlierTh() const {
    return _inlier_th;
  }
  inline float& inlierTh() {
    return _inlier_th;
  }

 protected:
 
  MeasurementStat errorAndJacobian(const Measurement& measurement_,
                                   ErrorType& error_,
                                   JacobianType& jacobian_,
                                   float& weight_,
                                   bool error_only_=false) const;
  
  Matrix6f updateH() const;

  Measurements _measurements;

  SensorOffset _estimate = SensorOffset::Identity();
  InformationMatrix _omega = InformationMatrix::Zero();

  Matrix6f _H = Matrix6f::Zero();
  Vector6f _b = Vector6f::Zero();

  SolverStat _stats;
  int   _iterations = 10;
  float _dumping = 0.f;
  float _inlier_th = 1.f;

  MEstimatorType mEstimator = std::bind(huber, std::placeholders::_1, std::placeholders::_2, 1.f); 

};

}  // namespace ca2lib