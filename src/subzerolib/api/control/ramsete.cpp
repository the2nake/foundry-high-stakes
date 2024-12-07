#include "subzerolib/api/control/ramsete.hpp"
#include "subzerolib/api/util/logging.hpp"
#include "subzerolib/api/util/math.hpp"

Ramsete::Ramsete(double i_b,
                 double i_zeta,
                 std::shared_ptr<Odometry> i_odom,
                 std::shared_ptr<TankChassis> i_chassis,
                 std::unique_ptr<Condition<double>> i_condition)
    : b(i_b), zeta(i_zeta), odom(std::move(i_odom)),
      chassis(std::move(i_chassis)), exit_condition(std::move(i_condition)) {}

void Ramsete::follow(const std::vector<trajectory_point_s> &trajectory,
                     uint timeout_ms) {
  this->exit_condition->reset();

  auto start = pros::millis();
  std::uint32_t prev = start;
  const auto prev_time = &prev;

  int i = 0;

  while (!this->exit_condition->is_met()) {
    this->settled = false;
    pose_s pose = this->odom->get_pose();

    if (pros::millis() - start > timeout_ms) {
      break;
    }

    if (pros::millis() - start > 1000.0 * trajectory[i].t) {
      ++i;
    }

    if (i >= trajectory.size()) {
      i = trajectory.size() - 1;
    }

    auto goal = trajectory[i];

    double vd = goal.v();
    double wd = in_rad(goal.vh);

    double e_xg = goal.x - pose.x;
    double e_yg = goal.y - pose.y;

    double h_cartesian = 0.5 * K_PI - pose.radians();

    // forward is positive x
    // left is positive y
    double e_x = e_xg * cos(h_cartesian) + e_yg * sin(h_cartesian);
    double e_y = e_xg * sin(h_cartesian) - e_yg * cos(h_cartesian);
    double e_theta = in_rad(shorter_turn(pose.heading(), goal.h));

    if (std::abs(vd) < K_EPSILON) {
      vd = 0.005 * e_x;
    }

    if (std::abs(wd) < K_EPSILON) {
      wd = 0.005 * e_theta;
    }

    double k = 2.0 * this->zeta * std::sqrt(wd * wd + this->b * vd * vd);
    double v = vd * cos(e_theta) + k * e_x;
    double w = wd + k * e_theta + this->b * vd * sin(e_theta) * e_y / e_theta;

    auto vels = this->chassis->get_wheel_vels(0, v, w);
    this->chassis->move_vels(vels);

    this->exit_condition->update(pose.dist(trajectory.back()));
    pros::Task::delay_until(prev_time, 10);
  }

  subzero::log("[i]: ramsete finished in %d ms", pros::millis() - start);
  this->settled = true;
  this->chassis->move_tank(0, 0);
}