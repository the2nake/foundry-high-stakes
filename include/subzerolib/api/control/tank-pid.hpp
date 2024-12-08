#pragma once
#include "subzerolib/api/chassis/tank-chassis.hpp"
#include "subzerolib/api/control/mtp-controller.hpp"
#include "subzerolib/api/logic/exit-condition.hpp"
#include "subzerolib/api/odometry/odometry.hpp"

class TankPID : public MtpController {
public:
  /// @param ichassis a TankChassis with velocity PID tuned
  /// @param id_pid pid controller tuned for difference in metres, output in
  /// velocity (m/s)
  /// @param ir_pid pid controller tuned for difference in degrees, output in
  /// angular velocity (rad/s)
  TankPID(std::shared_ptr<TankChassis> ichassis,
          std::shared_ptr<Odometry> iodom,
          std::unique_ptr<Condition<double>> ipos_exit,
          std::unique_ptr<PIDF> id_pid,
          std::unique_ptr<PIDF> ir_pid);

  /// @brief move the chassis in the direction of a target pose
  ///
  /// @param target the target pose
  /// @param linv the intended movement velocity
  void approach_pose(pose_s target, double linv = std::nan("")) override;

  /// @brief brake the chassis
  void stop() override;

  bool is_settled() override { return this->settled.load(); }

private:
  std::shared_ptr<TankChassis> chassis;
  std::shared_ptr<Odometry> odom;
  std::unique_ptr<Condition<double>> pos_exit;

  std::unique_ptr<PIDF> d_pid;
  std::unique_ptr<PIDF> r_pid;

  std::atomic<bool> settled = true;
};