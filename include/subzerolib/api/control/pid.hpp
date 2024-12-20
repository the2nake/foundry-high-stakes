#pragma once

#include "pros/rtos.hpp"
#include <atomic>
#include <cmath>
#include <functional>

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
  /// @param iff feedfoward function (takes and returns double)
  /// @returns a PIDF object
  PIDF(
      double ikp,
      double iki,
      double ikd,
      bool cut_integral = false,
      std::function<double(double)> iff = [](double ignore) { return 0.0; })
      : kp(ikp), ki(iki), kd(ikd), ff(iff), cut(cut_integral) {}

  PIDF(PIDF &&other)
      : kp(other.kp), ki(other.ki), kd(other.kd), ff(other.ff), cut(other.cut) {
  }

  /// @brief reset the controller
  ///
  /// affects output, total_error, prev_error, and last_update
  void reset();

  /// @brief update the controller with an error input
  ///
  /// update effects are time-sensitive, however behaviour is mostly uniform
  /// with varying update rates
  ///
  /// @param error the error to the target reading. usually calculated as
  /// (target - current)
  /// @param ff_input input to the feedfoward, oftentimes the target trying to
  /// be reached. nan disables feedforward.
  /// @returns the output of the controller
  double update(double error, double ff_input = std::nan(""));

  /// @brief get the output of the controller
  /// @returns the output value
  double get_output() { return output.load(); }

private:
  std::atomic<double> output;
  const double kp;
  const double ki;
  const double kd;
  const bool cut;
  std::function<double(double)> ff;

  std::atomic<double> prev_err = std::nan("");
  std::atomic<double> total_err = 0.0;

  uint32_t last_update = pros::millis();
};
