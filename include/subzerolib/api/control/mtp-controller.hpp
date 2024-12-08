#pragma once

#include "subzerolib/api/geometry/pose.hpp"

/// @brief class that provides movement algorithms with position-based movement
/// functions
///
/// implementations of apporach_pose need not work as a move to pose function on
/// their own; they simply must approach correctly given that the pose is
/// roughly aligned with the direction of the drive
class MtpController {
public:
  /// @brief move the chassis in the direction of a target pose
  ///
  /// @param target the target pose
  /// @param linv the intended movement velocity
  virtual void approach_pose(pose_s target, double linv = std::nan("")) = 0;

  virtual void move_to_pose(pose_s target, int timeout_ms = 6000);

  /// @brief brake the chassis
  virtual void stop() = 0;

  /// @brief check if the controller is settled
  virtual bool is_settled() = 0;
};
