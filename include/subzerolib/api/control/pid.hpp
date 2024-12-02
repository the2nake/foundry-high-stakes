#pragma once

#include "pros/rtos.hpp"
#include <atomic>
#include <cmath>

class PIDF {
public:
  /// @brief create a PIDF controller
  ///
  /// tuning parameters other than feedfoward cannot be changed after
  /// initialisation
  ///
  /// @param ikp proportional gain
  /// @param iki integral gain
  /// @param ikd derivative gain
  /// @param iff feedfoward constant
  /// @returns a PIDF object
  PIDF(double ikp, double iki, double ikd, double iff = 0)
      : kp(ikp), ki(iki), kd(ikd), ff(iff) {}

  PIDF(PIDF &&other) : kp(other.kp), ki(other.ki), kd(other.kd), ff(other.ff) {}

  /// @brief reset the controller
  ///
  /// affects output, total_error, prev_error, and last_update
  void reset();

  /// @brief changes the feedforward constant
  /// @param iff the new feedforward value
  void set_ff(double iff) {
    if (!std::isnan(iff)) {
      ff = iff;
    } else {
      ff = 0.0;
    }
  }

  /// @brief update the controller with an error input
  ///
  /// update effects are time-sensitive, however behaviour is mostly uniform
  /// with varying update rates
  ///
  /// @param error the error to the target reading. usually calculated as
  /// (target - current)
  /// @returns the output of the controller
  double update(double error);

  /// @brief get the output of the controller
  /// @returns the output value
  double get_output() { return output.load(); }

private:
  std::atomic<double> output;
  const double kp;
  const double ki;
  const double kd;
  double ff = 0.0;

  std::atomic<double> prev_err = std::nan("");
  std::atomic<double> total_err = 0.0;

  uint32_t last_update = pros::millis();
};
