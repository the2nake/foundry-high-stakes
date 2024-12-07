#pragma once

#include "subzerolib/api/chassis/chassis.hpp"
#include "subzerolib/api/control/pid.hpp"
#include "subzerolib/api/util/controls.hpp"

#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"
#include <map>
#include <memory>

class TankChassis : public Chassis {
public:
  enum class motor_pos_e { left, right };

  /// @brief specify a preference for combining linear and angular components
  /// @param rot_pref a value in the range [0.0, 1.0]
  void set_rot_pref(double i_rot_pref = 0.5) override;

  /// @brief move in relative reference frame
  ///
  /// @param x lateral component (unused)
  /// @param y vertical component
  /// @param r rotational component
  void move(double x, double y, double r) override;

  /// @brief move with a certain wheel velocity for each wheel
  ///
  /// error input is in RPM
  /// feedforward input is in RPM
  /// @param vels velocities for each wheel (left then right)
  void move_vels(std::vector<double> vels) override;

  /// @brief move with basic tank drive controls
  ///
  /// @param left left power (-1.0 -> 1.0)
  /// @param right right power (-1.0 -> 1.0)
  void move_tank(double left, double right);

  /// @brief generate wheel velocities for a given angular and linear velocity
  /// target
  ///
  /// @param vx (unused)
  /// @param vy linear velocity (m/s) in local frame's y
  /// @param ang angular velocity (rad/s)
  ///
  /// @returns a pair of velocities (left, right)
  std::vector<double> get_wheel_vels(double vx, double vy, double ang) override;

  std::vector<double> get_actual_vels() override;

  /// @brief get the maximum velocity
  /// @returns the linear velocity
  double get_max_vel() override;

  /// @brief get the maximum velocities of each wheel
  /// @returns a vector in the same order as get_wheel_velocities()
  std::vector<double> get_wheel_max() override;

private:
  TankChassis() {}
  void move_with_map();

  double circum;
  double track_width;
  double rot_pref;
  double lin_vel;

  std::map<TankChassis::motor_pos_e, control_components_s> volts = {};
  std::map<TankChassis::motor_pos_e, std::unique_ptr<pros::AbstractMotor> &>
      position_ptr_map = {};
  std::unique_ptr<pros::AbstractMotor> left = nullptr;
  std::unique_ptr<pros::AbstractMotor> right = nullptr;

  std::unique_ptr<PIDF> left_pid = nullptr;
  std::unique_ptr<PIDF> right_pid = nullptr;

public:
  class Builder {
  public:
    /// @brief specify a Motor for the chassis
    ///
    /// the chassis motor should be reversed such that positive voltage moves
    /// the drive forward
    ///
    /// @param position the motor position
    /// @param motor a unique pointer to the motor
    /// @returns a reference to the builder object
    Builder &with_motor(motor_pos_e position,
                        std::unique_ptr<pros::Motor> motor);
    /// @brief specify an AbstractMotor for the chassis
    ///
    /// the chassis motor should be reversed such that positive voltage moves
    /// the drive forward
    ///
    /// @param position the motor position
    /// @param motor a unique pointer to the motor
    /// @returns a reference to the builder object
    Builder &with_motor(motor_pos_e position,
                        std::unique_ptr<pros::AbstractMotor> motor);

    /// @brief specify pid controllers for both sides
    Builder &with_pid(std::unique_ptr<PIDF> left, std::unique_ptr<PIDF> right);

    /// @brief specify drive geometry
    /// @param i_circum distance for one revolution of the motors
    /// @param i_track_width distance between the centers of the two wheels
    /// across each other
    /// @returns a reference to the builder object
    Builder &with_geometry(double i_circum, double i_track_width);

    /// @brief specify a preference for combining linear and angular components
    /// @param i_rot_pref a value in the range [0.0, 1.0]
    /// @returns a reference to the builder object
    Builder &with_rot_pref(double i_rot_pref = 0.5);

    Builder &with_vel(double i_lin_vel);

    /// @brief creates the tank drive chassis object
    /// @returns a shared pointer to the created object
    std::shared_ptr<TankChassis> build();

  private:
    double b_track_width = 0.25;
    double b_circum = 0.22;

    double b_rot_pref = 0.5;
    double b_lin_vel = 1;

    std::unique_ptr<pros::AbstractMotor> b_left = nullptr;
    std::unique_ptr<pros::AbstractMotor> b_right = nullptr;

    std::unique_ptr<PIDF> b_left_pid = nullptr;
    std::unique_ptr<PIDF> b_right_pid = nullptr;
  };
};
