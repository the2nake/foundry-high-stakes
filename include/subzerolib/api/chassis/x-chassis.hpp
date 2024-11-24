#pragma once

#include "subzerolib/api/chassis/chassis.hpp"
#include "subzerolib/api/util/math.hpp"

#include "pros/abstract_motor.hpp"
#include <memory>

class XChassis : public Chassis {
public:
  /// @brief move in relative reference frame
  ///
  /// @param x lateral component
  /// @param y vertical component
  /// @param r rotational component
  void move(double x, double y, double r) override;

  /// @brief specify a preference for combining linear and angular components
  /// @param rot_pref a value in the range [0.0, 1.0]
  void set_rot_pref(double irot_pref) override {
    rot_pref = std::abs(irot_pref);
    clamp_val(rot_pref, 0.0, 1.0);
  }

  /// @brief get the maximum velocity
  /// @returns the linear velocity
  double get_max_vel() override;

  /// @brief generate wheel velocities for a given angular and linear velocity
  /// target
  ///
  /// @param vx linear velocity (m/s) in local frame's x
  /// @param vy linear velocity (m/s) in local frame's y
  /// @param ang angular velocity (rad/s)
  ///
  /// @returns a vector of velocities (lf, lb, rf, rb)
  std::vector<double> get_wheel_vels(double vx, double vy, double ang) override;

  /// @brief get the maximum velocities of each wheel
  /// @returns a vector in the same order as get_wheel_velocities()
  std::vector<double> get_wheel_max() override;

private:
  XChassis() {}

  double rot_pref = 0.5;
  double radius = 1;
  double lin_vel = 1;

  std::unique_ptr<pros::AbstractMotor> front_left;
  std::unique_ptr<pros::AbstractMotor> front_right;
  std::unique_ptr<pros::AbstractMotor> back_right;
  std::unique_ptr<pros::AbstractMotor> back_left;

public:
  class Builder {
  public:
    /// @brief specify motors for the chassis
    ///
    /// the chassis motors should be reversed such that positive voltage moves
    /// the drive forward
    ///
    /// @param ifront_left a unique pointer to the front left motor
    /// @param ifront_right a unique pointer to the front right motor
    /// @param iback_right a unique pointer to the back right motor
    /// @param iback_left a unique pointer to the back left motor
    /// @returns a reference to the builder object
    Builder &with_motors(std::unique_ptr<pros::AbstractMotor> ifront_left,
                         std::unique_ptr<pros::AbstractMotor> ifront_right,
                         std::unique_ptr<pros::AbstractMotor> iback_right,
                         std::unique_ptr<pros::AbstractMotor> iback_left);

    /// @brief specify a preference for combining linear and angular components
    /// @param irot_pref a value in the range [0.0, 1.0]
    /// @returns a reference to the builder object
    Builder &with_rot_pref(double irot_pref) {
      brot_pref = std::abs(irot_pref);
      clamp_val(brot_pref, 0.0, 1.0);
      return *this;
    }

    Builder &with_geometry(double iradius);

    Builder &with_vel(double ilin_vel);

    /// @brief creates the x drive chassis object
    /// @returns a shared pointer to the created object
    std::shared_ptr<XChassis> build();

  private:
    bool failed = false;
    double brot_pref = 0.5;
    double bradius = 1;
    double blin_vel = 1;
    std::unique_ptr<pros::AbstractMotor> bfront_left = nullptr;
    std::unique_ptr<pros::AbstractMotor> bfront_right = nullptr;
    std::unique_ptr<pros::AbstractMotor> bback_right = nullptr;
    std::unique_ptr<pros::AbstractMotor> bback_left = nullptr;
  };
};
