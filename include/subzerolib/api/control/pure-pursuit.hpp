#pragma once

#include "subzerolib/api/control/mtp-controller.hpp"
#include "subzerolib/api/geometry/pose.hpp"
#include "subzerolib/api/logic/exit-condition.hpp"
#include "subzerolib/api/odometry/odometry.hpp"

#include "pros/rtos.hpp"
#include <atomic>
#include <memory>
#include <vector>

class PurePursuit {
public:
  /// @brief creates a pure pursuit controller
  /// @param icontroller a shared pointer to a chassis controller
  /// @param iodom a shared pointer to an odometry provider
  /// @param ipos_exit_condition a shared pointer to an exit condition for
  /// position
  /// @returns the created controller object
  PurePursuit(std::shared_ptr<MtpController> ictrl,
              std::shared_ptr<Odometry> iodom,
              std::shared_ptr<Condition<double>> ipos_exit,
              int iresolution = 1);

  /// @brief follows the path described by the linear spline connecting the
  /// waypoints
  ///
  /// @param waypoints a vector of waypoints
  /// @param lookahead range for pure-pursuit smoothening
  /// @param ms_timeout maximum controller run time
  /// @param resolution number of physics steps per iteration, >= 1
  void follow(const std::vector<pose_s> &waypoints,
              double lookahead,
              int ms_timeout = 5000);

  /// @brief follows the path described by the linear spline connecting the
  /// waypoints without blocking
  ///
  /// @param waypoints a vector of waypoints
  /// @param lookahead range for pure-pursuit smoothening
  /// @param ms_timeout maximum controller run time
  /// @param resolution number of physics steps per iteration, >= 1
  void follow_async(const std::vector<pose_s> &waypoints,
                    double lookahead,
                    int timeout_ms = 5000);

  /// @brief stop the controller
  ///
  /// maximum lag of 10ms
  void stop();

  /// @brief checks if the motion is complete
  /// @returns if the motion has finished
  bool is_settled() { return settled.load(); }

private:
  void select_carrot(std::vector<pose_s>::const_iterator current,
                     double lookahead,
                     pose_s &carrot);
  int resolution = 1;

  std::shared_ptr<MtpController> ctrl;
  std::shared_ptr<Odometry> odom;
  std::shared_ptr<Condition<double>> pos_exit;

  std::atomic<bool> settled = true;
  pros::Mutex mutex;
};