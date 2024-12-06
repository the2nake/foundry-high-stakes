#pragma once

#include "subzerolib/api/control/mtp-controller.hpp"
#include "subzerolib/api/geometry/pose.hpp"
#include "subzerolib/api/logic/exit-condition.hpp"
#include "subzerolib/api/odometry/odometry.hpp"
#include "subzerolib/api/util/auto-updater.hpp"

#include "pros/rtos.hpp"
#include <atomic>
#include <memory>
#include <vector>

class PurePursuitController {
public:
  /// @brief creates a pure pursuit controller
  /// @param icontroller a shared pointer to a chassis controller
  /// @param iodom a shared pointer to an odometry provider
  /// @param ipos_exit_condition a shared pointer to an exit condition for
  /// position
  /// @returns the created controller object
  PurePursuitController(
      std::shared_ptr<MtpController> icontroller,
      std::shared_ptr<Odometry> iodom,
      std::shared_ptr<Condition<double>> ipos_exit_condition);

  /// @brief follows the path described by the linear spline connecting the
  /// waypoints
  ///
  /// @param waypoints a vector of waypoints
  /// @param lookahead range for pure-pursuit smoothening
  /// @param ms_timeout maximum controller run time
  /// @param resolution number of physics steps per iteration, >= 1
  void follow(std::vector<pose_s> iwaypoints,
              double lookahead,
              int ms_timeout = 5000,
              int iresolution = 1);
  // TODO: PurePursuitController::follow_async

  /// @brief stop the controller
  ///
  /// maximum lag of 10ms
  void stop();

  /// @brief checks if the motion is complete
  /// @returns if the motion has finished
  bool is_complete() { return motion_complete.load(); }

private:
  void select_carrot(pose_s pose, double lookahead, pose_s &carrot);
  std::vector<pose_s> waypoints;
  int resolution = 1;

  std::shared_ptr<MtpController> controller;
  std::shared_ptr<Odometry> odom;
  std::shared_ptr<Condition<double>> pos_exit_condition;

  std::atomic<bool> motion_complete = true;
  pros::Mutex mutex;
  std::unique_ptr<AutoUpdater<double>> pos_exit_condition_updater;
};