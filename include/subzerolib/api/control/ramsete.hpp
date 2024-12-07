#pragma once

#include "subzerolib/api/chassis/tank-chassis.hpp"
#include "subzerolib/api/geometry/trajectory-point.hpp"
#include "subzerolib/api/logic/exit-condition.hpp"
#include "subzerolib/api/odometry/odometry.hpp"

#include <atomic>
#include <memory>
#include <vector>

class Ramsete {
public:
  Ramsete(double i_b,
          double i_zeta,
          std::shared_ptr<Odometry> i_odom,
          std::shared_ptr<TankChassis> i_chassis,
          std::unique_ptr<Condition<double>> i_condition);

  void follow(const std::vector<trajectory_point_s> &trajectory,
              uint timeout_ms = 5000);

  bool is_settled() { return this->settled.load(); }

private:
  const double b;
  const double zeta;

  std::shared_ptr<Odometry> odom;
  std::shared_ptr<TankChassis> chassis;
  std::unique_ptr<Condition<double>> exit_condition;

  std::atomic<bool> settled;
};