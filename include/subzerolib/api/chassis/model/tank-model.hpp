#pragma once
#include "subzerolib/api/chassis/model/model.hpp"
#include <cmath>

struct TankModel : public Model {
  const double vel = 1.5;
  const double accel = 6.0;
  const double decel = 4.0;
  const double track_width = 0.25;

  const double drift = 0.0;

  // TODO: simulate and graph effect of drift on trajectory

  TankModel(double i_vel, double i_accel, double i_decel, double i_track_width)
      : vel(std::abs(i_vel)), accel(std::abs(i_accel)),
        decel(std::abs(i_decel)), track_width(std::abs(i_track_width)) {}

  ModelConstraints get_constraints(double curvature) const override;
};