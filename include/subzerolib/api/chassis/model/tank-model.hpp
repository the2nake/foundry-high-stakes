#pragma once
#include "subzerolib/api/chassis/model/model.hpp"
#include <cmath>

struct TankModel : public Model {
  const double vel;
  const double accel;
  const double decel;
  const double track_width;

  const double drift;

  // TODO: simulate and graph effect of drift on trajectory

  TankModel(double i_vel, double i_accel, double i_decel, double i_track_width, double i_drift = 0.0)
      : vel(std::abs(i_vel)), accel(std::abs(i_accel)),
        decel(std::abs(i_decel)), track_width(std::abs(i_track_width)), drift(i_drift) {}

  ModelConstraints get_constraints(double curvature) const override;
};