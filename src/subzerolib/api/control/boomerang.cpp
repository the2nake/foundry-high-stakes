#include "subzerolib/api/control/boomerang.hpp"

Boomerang::Boomerang(std::shared_ptr<TankChassis> ictrl,
                     std::shared_ptr<Odometry> iodom,
                     std::shared_ptr<Condition<double>> ipos_exit,
                     std::unique_ptr<PIDF> id_pid,
                     std::unique_ptr<PIDF> ir_pid,
                     double ilead)
    : chassis(std::move(ictrl)), odom(std::move(iodom)),
      pos_exit(std::move(ipos_exit)), d_pid(std::move(id_pid)),
      r_pid(std::move(ir_pid)), lead(std::abs(ilead)) {}

void Boomerang::approach_pose(pose_s target, double linv) {
  auto pose = odom->get_pose();
  this->pos_exit->update(pose.dist(target));
  this->settled = this->pos_exit->is_met();

  double h_cartesian = in_rad(90 - target.heading());
  point_s carrot = point_s{target} -
                   this->lead * point_s{cos(h_cartesian), sin(h_cartesian)};

  auto e_x = carrot.x - pose.x;
  auto e_y = carrot.y - pose.y;
  double e_theta = shorter_turn(pose.heading(), pose.heading_to(carrot));
  double e_rot =
      std::min(shorter_turn(pose.heading(), pose.heading_to(carrot) + 180),
               e_theta,
               [](const double &a, const double &b) {
                 return std::abs(a) < std::abs(b);
               });

  chassis->move_vels(chassis->get_wheel_vels(
      0,
      cos(in_rad(e_theta)) * d_pid->update(std::hypot(e_x, e_y), linv),
      r_pid->update(e_rot)));
}

void Boomerang::brake() {
  this->chassis->move(0, 0, 0);
  this->pos_exit->reset();
  this->settled = true;
}
