#include "subzerolib/api/control/ramsete.hpp"
#include "subzerolib/api/util/logging.hpp"
#include "subzerolib/api/util/math.hpp"
#include <fstream>

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

  std::ofstream out("/usd/tuning.txt", std::ios::out);

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

    /* NO FEEDBACK
    auto vels = this->chassis->get_wheel_vels(0, goal.v(), in_rad(goal.vh));
    if (vels[0] > this->chassis->get_max_vel() ||
        vels[1] > this->chassis->get_max_vel()) {
      subzero::error("[w] ramsete: invalid profile %f %f", vels[0], vels[1]);
    }
                             */
    // trajectory_point_s goal =
    //     closest_point_on_path<trajectory_point_s>(trajectory, pose);

    double vd = goal.v();
    double wd = in_rad(goal.vh);

    double err_x_global = goal.x - pose.x;
    double err_y_global = goal.y - pose.y;

    double angle_cart = 0.5 * K_PI - pose.radians();

    // forward is positive x
    // left is positive y
    double err_x =
        err_x_global * cos(angle_cart) + err_y_global * sin(angle_cart);
    double err_y =
        err_x_global * sin(angle_cart) - err_y_global * cos(angle_cart);
    double err_theta = in_rad(shorter_turn(pose.heading(), goal.h));

    if (std::abs(vd) < K_EPSILON) {
      vd = 0.01 * err_x;
    }

    if (std::abs(wd) < K_EPSILON) {
      wd = 0.01 * err_theta;
    }

    double k = 2.0 * this->zeta * std::sqrt(wd * wd + this->b * vd * vd);
    double v = vd * cos(err_theta) + k * err_x;
    double w =
        wd + k * err_theta + this->b * vd * sin(err_theta) * err_y / err_theta;

    auto vels = this->chassis->get_wheel_vels(0, v, w);
    this->chassis->move_vels(vels);
    subzero::print(
        8, "ramsete %d %f %f %f", pros::millis(), goal.x, goal.y, goal.h);
    subzero::print(9, "ramsete err %f %f %f", err_x, err_y, in_deg(err_theta));

    char msg[100];
    snprintf(msg,
             100,
             "%d %f %f\n",
             pros::millis(),
             chassis->get_actual_vels()[0],
             vels[0]);
    if (out.is_open()) {
      out << msg;
    }

    this->exit_condition->update(pose.dist(trajectory.back()));
    pros::Task::delay_until(prev_time, 10);
  }
  out.close();

  subzero::log("[i]: ramsete finished in %d ms", pros::millis() - start);
  this->settled = true;
  this->chassis->move_tank(0, 0);
}