#include "subzerolib/api/control/tank-pid.hpp"

TankPID::TankPID(std::shared_ptr<TankChassis> ictrl,
                 std::shared_ptr<Odometry> iodom,
                 std::unique_ptr<Condition<double>> ipos_exit,
                 std::unique_ptr<PIDF> id_pid,
                 std::unique_ptr<PIDF> ir_pid)
    : chassis(std::move(ictrl)), odom(std::move(iodom)),
      pos_exit(std::move(ipos_exit)), d_pid(std::move(id_pid)),
      r_pid(std::move(ir_pid)) {}

void TankPID::approach_pose(pose_s target, double linv) {
  auto pose = odom->get_pose();
  auto dist = pose.dist(target);
  this->pos_exit->update(dist);
  this->settled = this->pos_exit->is_met();

  if (r_pid != nullptr && d_pid != nullptr) {
    double h_to_target = pose.heading_to(target);
    double e_theta = shorter_turn(pose.heading(), h_to_target);
    double e_rot = aligned_turn(pose.heading(), h_to_target);

    double k_lin = sgn(cos(in_rad(e_theta)));
    /*
    auto path_angle = std::abs(
        shorter_turn(ghost.heading_to(pose), ghost.heading_to(target)));
    if (path_angle < 90.0) {
      k_lin *= 0.3;
    } else {
      k_lin *= 0.7 + 0.3 * std::sin(path_angle);
    }
    */
    auto vy = k_lin * d_pid->update(pose.dist(target), linv);

    auto lin_vels = chassis->get_wheel_vels(0, vy, 0);
    auto ang_vels = chassis->get_wheel_vels(0, 0, r_pid->update(e_rot));
    double overshoot = std::abs(lin_vels[0]) + std::abs(ang_vels[0]) -
                       this->chassis->get_max_vel();
    if (overshoot > 0) {
      if (lin_vels[0] > 0) {
        lin_vels[0] -= overshoot;
        lin_vels[1] -= overshoot;
      } else {
        lin_vels[0] += overshoot;
        lin_vels[1] += overshoot;
      }
    }
    chassis->move_vels({lin_vels[0] + ang_vels[0], lin_vels[1] + ang_vels[1]});
  } else {
    // use constant curvature
    double h_cartesian = 0.5 * K_PI - pose.radians();
    auto eg = point_s{target} - point_s{pose};
    double e_x = eg.x * cos(h_cartesian) + eg.y * sin(h_cartesian);
    double e_y = eg.x * sin(h_cartesian) - eg.y * cos(h_cartesian);
    double r = std::hypot(e_x, e_y) * 0.5 / e_y;

    chassis->move_vels(chassis->get_wheel_vels(0, linv, linv / r));
  }
}

void TankPID::stop() {
  this->chassis->move_vels({0, 0});
  this->pos_exit->reset();
  this->settled = true;
}
