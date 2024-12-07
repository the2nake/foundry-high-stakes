#include "subzerolib/api/control/pid.hpp"
#include "pros/rtos.hpp"
#include <cmath>

void PIDF::reset() {
  last_update = pros::millis();
  prev_err = std::nan("");
  total_err = 0.0;
  output = 0.0;
}

double PIDF::update(double error, double ff_input) {
  uint32_t now = pros::millis();
  double dt = (now - last_update) / 1000.0; // in seconds
  if (std::isnan(error) || std::isinf(error)) {
    output = 0.0;
    return 0.0;
  }

  bool not_same_sgn = (std::signbit(prev_err) != std::signbit(error));
  if (std::isnan(total_err) || (!std::isfinite(total_err)) ||
      (this->cut && not_same_sgn)) {
    total_err = 0.0;
  }
  total_err += error * dt;
  double p = kp * error;
  double i = ki * total_err;
  double d = 0.0;
  if ((!std::isnan(prev_err)) && (std::abs(dt) > 0.001)) {
    d = kd * (error - prev_err) / dt;
  }
  output = (!std::isnan(ff_input) ? ff(ff_input) : 0) + p + i + d;
  prev_err = error;
  last_update = now;
  return output;
}