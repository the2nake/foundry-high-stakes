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
  auto dist = pose.dist(target);
  this->pos_exit->update(dist);
  this->settled = this->pos_exit->is_met();

  double target_rad = in_rad(90 - target.heading());
  point_s carrot =
      point_s{target} -
      this->lead * dist * point_s{cos(target_rad), sin(target_rad)};

  double h_to_carrot = pose.heading_to(carrot);
  double e_theta = shorter_turn(pose.heading(), h_to_carrot);
  double e_rot = aligned_turn(pose.heading(), h_to_carrot);

  chassis->move_vels(chassis->get_wheel_vels(
      0,
      cos(in_rad(e_theta)) * d_pid->update(pose.dist(carrot), linv),
      r_pid->update(e_rot)));
}

void Boomerang::stop() {
  this->chassis->move_vels({0, 0});
  this->pos_exit->reset();
  this->settled = true;
}
