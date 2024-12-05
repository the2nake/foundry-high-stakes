#include "subzerolib/api/control/holo-chassis-pid.hpp"
#include "pros/misc.hpp"
#include "subzerolib/api/logic/exit-condition.hpp"
#include "subzerolib/api/util/auto-updater.hpp"
#include "subzerolib/api/util/logging.hpp"
#include "subzerolib/api/util/math.hpp"

void HoloChassisPID::approach_pose(pose_s target, double linv) {
  auto pose = odom->get_pose();
  x_pid->update(target.x - pose.x);
  y_pid->update(target.y - pose.y);
  r_pid->update(shorter_turn(pose.h, target.h, 360.0));

  point_s vel{x_pid->get_output(), y_pid->get_output()};
  vel = rotate_acw(vel.x, vel.y, pose.h);

  chassis->move(vel.x, vel.y, r_pid->get_output());
}

void HoloChassisPID::move_to_pose(pose_s goal) {
  std::shared_ptr<ExitCondition<double>> cond{
      new ExitCondition<double>{{0, 0.02}, 200}
  };
  AutoUpdater<double> updater(
      [cond](double val) { cond->update(val); },
      [goal, this]() -> double { return odom->get_pose().dist(goal); });
  updater.start(10);
  auto init_status = pros::competition::get_status();
  std::uint32_t time = pros::millis();
  std::uint32_t *ptr = &time;
  while (!cond->is_met() && pros::competition::get_status() == init_status) {
    approach_pose(goal);
    pros::Task::delay_until(ptr, 10);
  }
  brake();
  updater.stop();
  subzero::log("[i]: pid to (%.2f, %.2f) @ %.0f done", goal.x, goal.y, goal.h);
}

HoloChassisPID::Builder &
HoloChassisPID::Builder::with_chassis(std::shared_ptr<Chassis> ichassis) {
  if (ichassis != nullptr) {
    bchassis = std::move(ichassis);
  }
  return *this;
}

HoloChassisPID::Builder &
HoloChassisPID::Builder::with_odom(std::shared_ptr<Odometry> iodom) {
  if (iodom != nullptr) {
    bodom = std::move(iodom);
  }
  return *this;
}

HoloChassisPID::Builder &
HoloChassisPID::Builder::with_pid(HoloChassisPID::pid_dimension_e dimension,
                                  PIDF &pid) {
  auto ptr = std::unique_ptr<PIDF>{new PIDF(std::move(pid))};
  switch (dimension) {
  case HoloChassisPID::pid_dimension_e::x:
    bx_pid = std::move(ptr);
    break;
  case HoloChassisPID::pid_dimension_e::y:
    by_pid = std::move(ptr);
    break;
  case HoloChassisPID::pid_dimension_e::r:
    br_pid = std::move(ptr);
    break;
  }
  return *this;
}

std::shared_ptr<HoloChassisPID> HoloChassisPID::Builder::build() {
  HoloChassisPID *controller = new HoloChassisPID();
  if (bchassis == nullptr || bodom == nullptr || bx_pid == nullptr ||
      by_pid == nullptr || br_pid == nullptr) {
    return nullptr;
  }

  controller->chassis = bchassis;
  controller->odom = bodom;
  controller->x_pid = std::move(bx_pid);
  controller->y_pid = std::move(by_pid);
  controller->r_pid = std::move(br_pid);

  return std::shared_ptr<HoloChassisPID>(controller);
}
