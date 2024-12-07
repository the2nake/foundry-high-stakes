#pragma once

#include <vector>

class Chassis {
public:
  /// @brief move in relative reference frame
  ///
  /// @param x lateral component
  /// @param y vertical component
  /// @param r rotational component
  virtual void move(double x, double y, double r) = 0;

  /// @brief move with a certain wheel velocity for each wheel
  virtual void move_vels(std::vector<double> vels) = 0;

  /// @brief specify a preference for combining linear and angular components
  /// @param rot_pref a value in the range [0.0, 1.0]
  virtual void set_rot_pref(double irot_pref = 0.5) = 0;

  /// @brief get the maximum velocity
  /// @returns the linear velocity
  virtual double get_max_vel() = 0;

  /// @brief generate wheel velocities for a given angular and linear velocity
  /// target
  ///
  /// @param vx linear velocity (m/s) in local frame's x
  /// @param vy linear velocity (m/s) in local frame's y
  /// @param ang angular velocity (rad/s)
  ///
  /// @returns a vector of velocities
  virtual std::vector<double>
  get_wheel_vels(double vx, double vy, double ang) = 0;

  virtual std::vector<double> get_actual_vels() = 0;

  /// @brief get the maximum velocities of each wheel
  /// @returns a vector in the same order as get_wheel_velocities()
  virtual std::vector<double> get_wheel_max() = 0;

protected:
  Chassis() {}
};
