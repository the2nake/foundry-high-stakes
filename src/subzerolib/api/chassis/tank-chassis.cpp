#include "subzerolib/api/chassis/tank-chassis.hpp"
#include "subzerolib/api/util/helper.hpp"

void TankChassis::set_rot_pref(double i_rot_pref) {
  clamp<double>(i_rot_pref, 0, 1);
  rot_pref = i_rot_pref;
}

void TankChassis::move(double x, double y, double r) {
  // do not clamp velocities, keep original ratios
  insert_or_modify(volts, TankChassis::motor_pos_e::left, {y, r});
  insert_or_modify(volts, TankChassis::motor_pos_e::right, {y, -r});
  // TODO: uncomment balance_mapped_vels(volts, 1.0, rot_pref);
  move_with_map();
}

void TankChassis::move_with_map() {
  for (auto &pair : volts) {
    position_ptr_map.at(pair.first)->move_voltage(12000 * pair.second.sum());
  }
}

std::vector<double>
TankChassis::get_wheel_vels(double vx, double vy, double ang) {
  // net = vl - vr (m/s)
  // net / (0.5 * track_width) = ang (1/s)
  // ang = (vl - vr) / (0.5 * track_width)

  // 0.5 * ang * track_width = vl - vr
  // 2 * vy = vl + vr
  //   vr = 2 * vy - vl
  // 0.5 * ang * track_width + 2 * vy = 2 * vl
  // 0.25 * ang * track_width + vy = vl

  double vl = 0.25 * ang * track_width + vy;
  double vr = 2 * vy - vl;

  return {vl, vr};
}

double TankChassis::get_max_vel() { return lin_vel; }
std::vector<double> TankChassis::get_wheel_max() { return {lin_vel, lin_vel}; }

TankChassis::Builder &
TankChassis::Builder::with_motor(motor_pos_e position,
                                 std::unique_ptr<pros::Motor> motor) {
  std::unique_ptr<pros::AbstractMotor> ptr(std::move(motor));
  return with_motor(position, std::move(ptr));
}

TankChassis::Builder &
TankChassis::Builder::with_motor(motor_pos_e position,
                                 std::unique_ptr<pros::AbstractMotor> motor) {
  if (motor == nullptr) {
    return *this;
  }
  if (position == motor_pos_e::left && motor) {
    b_left = std::move(motor);
  } else {
    b_right = std::move(motor);
  }
  return *this;
}

TankChassis::Builder &
TankChassis::Builder::with_geometry(double i_track_width) {
  b_track_width = std::abs(i_track_width);
  return *this;
}

TankChassis::Builder &TankChassis::Builder::with_rot_pref(double irot_pref) {
  clamp(irot_pref, 0.0, 1.0);
  b_rot_pref = irot_pref;
  return *this;
}

TankChassis::Builder &TankChassis::Builder::with_vel(double ilin_vel) {
  b_lin_vel = std::abs(ilin_vel);
  return *this;
}

std::shared_ptr<TankChassis> TankChassis::Builder::build() {
  auto chassis = new TankChassis();

  if (b_left == nullptr || b_right == nullptr) {
    return nullptr;
  }

  chassis->left = std::move(b_left);
  chassis->right = std::move(b_right);

  chassis->track_width = b_track_width;
  chassis->rot_pref = b_rot_pref;
  chassis->lin_vel = b_lin_vel;

  chassis->position_ptr_map = {
      { motor_pos_e::left,  chassis->left},
      {motor_pos_e::right, chassis->right}
  };
  return std::shared_ptr<TankChassis>(chassis);
}
