#include "subsystems/arm.hpp"

void Arm::score() {
  if (this->target.load() == target_e::accept) {
    this->target = target_e::score;
  }
}

void Arm::move_intake(double volts) {
  if (this->target.load() == target_e::accept) {
    this->intake->move_voltage(volts * 1000);
  }
}

void Arm::override_intake(double volts) {
  this->target = target_e::none;
  this->intake->move_voltage(volts * 1000);
}

void Arm::stop_intake() {
  if (this->target.load() == target_e::none) {
    this->target = target_e::recover;
  }
  this->intake->brake();
}

void Arm::execute() {
  switch (this->target.load()) {
  case target_e::none:
    return;
  case target_e::score:
    if (this->pot.get_angle() > target_pos() - 5) {
      this->target = target_e::recover;
    }
    break;
  case target_e::recover:
    if (std::abs(this->pot.get_angle() - target_pos()) < 4) {
      this->target = target_e::accept;
    }
    break;
  case target_e::accept:
    break;
  }

  // ! do not use shorter_turn
  this->wrist_ctrl->update(target_pos() - this->pot.get_angle());
  this->wrist->move_voltage(wrist_ctrl->get_output() * 1000);
}