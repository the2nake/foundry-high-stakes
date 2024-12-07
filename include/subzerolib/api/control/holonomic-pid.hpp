#pragma once

#include "subzerolib/api/chassis/chassis.hpp"
#include "subzerolib/api/control/mtp-controller.hpp"
#include "subzerolib/api/control/pid.hpp"
#include "subzerolib/api/logic/exit-condition.hpp"
#include "subzerolib/api/odometry/odometry.hpp"

#include <memory>

class HolonomicPID : public MtpController {
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

  /// @brief brake the chassis
  void brake() override;

  bool is_settled() override { return this->settled.load(); }

private:
  std::shared_ptr<Chassis> chassis = nullptr;
  std::shared_ptr<Odometry> odom = nullptr;
  std::shared_ptr<Condition<double>> pos_exit = nullptr;

  std::unique_ptr<PIDF> x_pid = nullptr;
  std::unique_ptr<PIDF> y_pid = nullptr;
  std::unique_ptr<PIDF> r_pid = nullptr;

  std::atomic<bool> settled = true;

  HolonomicPID() {}

public:
  class Builder {
  public:
    Builder &with_chassis(std::shared_ptr<Chassis> ichassis);

    Builder &with_odom(std::shared_ptr<Odometry> iodom);

    Builder &with_pos_exit(std::shared_ptr<Condition<double>> ipos_exit);

    /// @brief adds a positional PID controller
    Builder &with_pid(pid_dimension_e dimension, PIDF &pid);

    std::shared_ptr<HolonomicPID> build();

  private:
    std::shared_ptr<Chassis> bchassis = nullptr;
    std::shared_ptr<Odometry> bodom = nullptr;
    std::shared_ptr<Condition<double>> bpos_exit = nullptr;

    std::unique_ptr<PIDF> bx_pid = nullptr;
    std::unique_ptr<PIDF> by_pid = nullptr;
    std::unique_ptr<PIDF> br_pid = nullptr;
  };
};
