#pragma once

#include <cmath>

struct ModelConstraints {
  const double max_vel;
  const double max_accel;
  const double max_decel;

  ModelConstraints(double i_vel, double i_accel, double i_decel)
      : max_vel(std::abs(i_vel)), max_accel(std::abs(i_accel)),
        max_decel(std::abs(i_decel)) {}
};

struct Model {
  virtual ModelConstraints get_constraints(double curvature) const = 0;
};