#include "subzerolib/api/control/holonomic-pid.hpp"
#include "subzerolib/api/logic/exit-condition.hpp"
#include "subzerolib/api/util/math.hpp"

void HolonomicPID::approach_pose(pose_s target, double linv) {
  auto pose = odom->get_pose();
  this->pos_exit->update(pose.dist(target));
  this->settled = this->pos_exit->is_met();

  auto e_x = target.x - pose.x;
  auto e_y = target.y - pose.y;

  x_pid->update(e_x, linv * e_x / std::hypot(e_x, e_y));
  y_pid->update(e_y, linv * e_y / std::hypot(e_x, e_y));
  r_pid->update(shorter_turn(pose.h, target.h, 360.0));

  point_s vel{x_pid->get_output(), y_pid->get_output()};
  vel = rotate_acw(vel.x, vel.y, pose.h);

  chassis->move(vel.x, vel.y, r_pid->get_output());
}

void HolonomicPID::stop() {
  this->chassis->move(0, 0, 0);
  this->pos_exit->reset();
  this->settled = true;
}

HolonomicPID::Builder &
HolonomicPID::Builder::with_chassis(std::shared_ptr<Chassis> ichassis) {
  if (ichassis != nullptr) {
    bchassis = std::move(ichassis);
  }
  return *this;
}

HolonomicPID::Builder &
HolonomicPID::Builder::with_odom(std::shared_ptr<Odometry> iodom) {
  if (iodom != nullptr) {
    bodom = std::move(iodom);
  }
  return *this;
}
HolonomicPID::Builder &HolonomicPID::Builder::with_pos_exit(
    std::shared_ptr<Condition<double>> ipos_exit) {
  if (ipos_exit != nullptr) {
    bpos_exit = std::move(ipos_exit);
  }
  return *this;
}

HolonomicPID::Builder &
HolonomicPID::Builder::with_pid(HolonomicPID::pid_dimension_e dimension,
                                  PIDF &pid) {
  auto ptr = std::unique_ptr<PIDF>{new PIDF(std::move(pid))};
  switch (dimension) {
  case HolonomicPID::pid_dimension_e::x:
    bx_pid = std::move(ptr);
    break;
  case HolonomicPID::pid_dimension_e::y:
    by_pid = std::move(ptr);
    break;
  case HolonomicPID::pid_dimension_e::r:
    br_pid = std::move(ptr);
    break;
  }
  return *this;
}

std::shared_ptr<HolonomicPID> HolonomicPID::Builder::build() {
  HolonomicPID *controller = new HolonomicPID();
  if (bchassis == nullptr || bodom == nullptr || bx_pid == nullptr ||
      by_pid == nullptr || br_pid == nullptr) {
    return nullptr;
  }

  controller->chassis = bchassis;
  controller->odom = bodom;
  controller->pos_exit = bpos_exit;
  controller->x_pid = std::move(bx_pid);
  controller->y_pid = std::move(by_pid);
  controller->r_pid = std::move(br_pid);

  return std::shared_ptr<HolonomicPID>(controller);
}
