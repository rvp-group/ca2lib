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

#include "ca2lib/solver/solver.h"
#include <numeric>
#include <algorithm>
#include <random>
#include <iostream>

namespace ca2lib {

std::ostream& operator<<(std::ostream& os, const IterationStat& istat_) {
  os << "it= " << istat_.iteration_number
     << "; num_inliers= " << istat_.num_inliers
     << "; chi_inliers= " << istat_.chi_inliers
     << "; num_outliers= " << istat_.num_outliers 
     << "; chi_outliers= " << istat_.chi_outliers;
  return os;
}

std::ostream& operator<<(std::ostream& os, const SolverStat& stats_) {
  using namespace std;
  os << "STATS: begin" << endl;
  for (const auto& istat : stats_) {
    os << istat << endl;
  }
  os << "STATS: end" << endl;
  return os;
}

MeasurementStat::Status huber(const float& chi_, float& weight_, float error_threshold_) {
  weight_ = 1.f;
  MeasurementStat::Status status;
  if (chi_ > error_threshold_) {
    weight_ = sqrt(error_threshold_ / chi_);
    status = MeasurementStat::Status::Kernelized;
  }
  status = MeasurementStat::Status::Inlier;
  return status;
}

Matrix6f Solver::updateH() const {
  Matrix6f H;
  for (const auto& m: _measurements) {
    ErrorType error;
    JacobianType J;
    float w;
    MeasurementStat m_stat = errorAndJacobian(m, error, J, w);
    H += J.transpose() * J;
  }
  return H;
}

bool Solver::compute() {

  if(_measurements.size() < 3)
    return false;

  for (int i = 0; i < _iterations; ++i) {
    _H = Matrix6f::Identity();
    _b = Vector6f::Zero();  
    _H *= _dumping;

    IterationStat stat{};
    stat.iteration_number = i+1;

    for (const auto& m: _measurements) {
      ErrorType error;
      JacobianType J;
      float weight;
      MeasurementStat m_stat = errorAndJacobian(m, error, J, weight);

      if(m_stat.chi > _inlier_th) {
        m_stat.status = MeasurementStat::Status::Outlier;
      }

      stat.measurement_stats.insert({m.id, m_stat});

      if(m_stat.status != MeasurementStat::Status::Outlier) {
        stat.num_inliers++;
        stat.chi_inliers += error.dot(error) * weight;
      } else {
        stat.num_outliers++;
        stat.chi_outliers += error.dot(error);
        continue;
      }

      _H += J.transpose() * J * weight;
      _b += J.transpose() * error * weight;
    }

    _stats.push_back(stat);

    if (stat.num_inliers < 3)
      return false;

    Vector6f delta_X = _H.colPivHouseholderQr().solve(-_b);
    _estimate =  v2t(delta_X) * _estimate;
    _omega = updateH();
  }

  return true;
}

MeasurementStat Solver::errorAndJacobian(const Measurement& measurement_,
                                 ErrorType& error_,
                                 JacobianType& jacobian_,
                                 float& weight_,
                                 bool error_only_) const {
  Plane moving = _estimate * measurement_.from;
  error_ = moving - measurement_.to;
  float chi = error_.dot(error_);
  MeasurementStat::Status status = mEstimator(chi, weight_);

  MeasurementStat m_stat;
  m_stat.status = status;
  m_stat.chi = chi;

  if(error_only_)
    return m_stat;

  jacobian_ = JacobianType::Zero();
  jacobian_.block<3,3>(0,3) = - skew(moving.normal());
  jacobian_.block<1,3>(3,0) = moving.normal().transpose();
  
  return m_stat;
}

}  // namespace ca2lib
