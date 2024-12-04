#include "arm.hpp"

void Arm::score() {
  if (this->enc_arm->get_angle() < 142.0) {
  }
  this->wrist_target = 260.0;
  this->wrist_vel = 60.0;
  this->arm_target = 149.0; // TODO: an actual value (nullopt to disable)

  // arm top = 240
  // wrist top = 200 -> 146
}

void Arm::update() {
  if (this->wrist->get_position() > 255.0) {
    this->wrist_target = 0.0;
    this->wrist_vel = 100.0;
    this->arm_target = 140.0;
  } else if (std::abs(this->wrist->get_position()) < 3.0 &&
             this->wrist_target < 259.0) {
    this->arm_target = std::nullopt;
  }
}

void Arm::execute() {
  update();

  if (wrist_target.has_value()) {
    this->wrist->move_absolute(wrist_target.value(), this->wrist_vel);
  }

  if (arm_target.has_value()) {
    // control loop
    this->intake->move_voltage(-this->arm_ctrl.update(
        arm_target.value() - this->enc_arm->get_angle() / 100.0));

  } else {
    this->arm_ctrl.reset();
  }
}

void Arm::move_intake(int v) {
  if (!arm_target.has_value()) {
    this->intake->move(v);
  }
}