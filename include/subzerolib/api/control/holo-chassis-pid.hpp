#pragma once

#include "subzerolib/api/chassis/chassis.hpp"
#include "subzerolib/api/control/mtp-controller.hpp"
#include "subzerolib/api/control/pid.hpp"
#include "subzerolib/api/odometry/odometry.hpp"

#include <memory>

class HoloChassisPID : public MtpController {
public:
  enum class pid_dimension_e {
    x,
    y,
    r,
  };

  /// @brief move the chassis in the direction of a target pose
  ///
  /// linear velocity is ignored
  ///
  /// @param target the target pose
  /// @param linv ignored paramter, set to nan by default
  void approach_pose(pose_s target, double linv = std::nan("")) override;

  void move_to_pose(pose_s target) override;

  /// @brief brake the chassis
  void brake() override { chassis->move(0, 0, 0); }

private:
  std::shared_ptr<Chassis> chassis = nullptr;
  std::shared_ptr<Odometry> odom = nullptr;
  std::unique_ptr<PIDF> x_pid = nullptr;
  std::unique_ptr<PIDF> y_pid = nullptr;
  std::unique_ptr<PIDF> r_pid = nullptr;

  HoloChassisPID() {}

public:
  class Builder {
  public:
    Builder &with_chassis(std::shared_ptr<Chassis> ichassis);

    Builder &with_odom(std::shared_ptr<Odometry> iodom);

    /// @brief adds a positional PID controller
    Builder &with_pid(pid_dimension_e dimension, PIDF &pid);

    std::shared_ptr<HoloChassisPID> build();

  private:
    std::shared_ptr<Chassis> bchassis = nullptr;
    std::shared_ptr<Odometry> bodom = nullptr;
    std::unique_ptr<PIDF> bx_pid = nullptr;
    std::unique_ptr<PIDF> by_pid = nullptr;
    std::unique_ptr<PIDF> br_pid = nullptr;
  };
};
