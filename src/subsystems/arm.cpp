#include "arm.hpp"
// #include "subzerolib/api/util/logging.hpp"

void Arm::score() {
  if (this->state == state_e_t::ready_m) {
    this->state = state_e_t::score_m;
  } else if (this->state == state_e_t::ready_w) {
    this->state = state_e_t::score_w;
  }
}

void Arm::switch_mode_to_wall(bool mode) {
  if (mode && this->state == state_e_t::ready_m) {
    this->state = state_e_t::trans_to_w;
  }

  if (!mode && this->state == state_e_t::ready_w) {
    this->state = state_e_t::trans_to_m;
  }
}

void Arm::toggle_wall_mode() {
  if (this->state == state_e_t::ready_m) {
    this->state = state_e_t::trans_to_w;
  }
  if (this->state == state_e_t::ready_w) {
    this->state = state_e_t::trans_to_m;
  }
}

// updates the targets and the state
void Arm::update() {
  switch (this->state) {
  case state_e_t::recover:
    this->wrist_vel = 100.0;
    if (this->enc_arm->get_deg() > 200) {
      this->wrist_target = 191.0;
    } else if (this->enc_arm->get_deg() > 144.0) {
      this->wrist_target = 30.0;
    } else {
      this->wrist_target = 0.0;
    }

    this->arm_target = 140.0;

    if (this->enc_arm->get_deg() < 142 && this->wrist->get_position() < 3.0) {
      this->state = state_e_t::accept;
      this->update();
    }
    break;
  case state_e_t::accept:
    this->arm_target = std::nullopt;

    this->state = state_e_t::ready_m;
    this->update();
    break;
  case state_e_t::ready_m:
    this->wrist_target = 0.0;
    this->arm_target = std::nullopt;
    break;
  case state_e_t::score_m:
    this->wrist_target = 260.0;
    this->wrist_vel = 60.0;
    this->arm_target = 145.5;

    if (this->wrist->get_position() > 257.0) {
      this->state = state_e_t::recover;
      this->update();
    }
    break;
  case state_e_t::trans_to_w:
    this->wrist_vel = 50.0;
    this->wrist_target = 70.0;

    if (this->wrist->get_position() >= 67.0) {
      this->arm_target = 242.5;
    }
    if (this->enc_arm->get_deg() > 160.0) {
      this->wrist_target = 210.0;
    }

    if (this->wrist->get_position() >= 190.0 &&
        this->enc_arm->get_deg() >= 242.3) {
      this->state = state_e_t::ready_w;
      this->update();
    }
    break;
  case state_e_t::ready_w:
    this->arm_target = 242.5;
    this->wrist_target = 210.0;
    break;
  case state_e_t::score_w:
    this->arm_target = 242.5;
    this->wrist_target = 110.0;
    this->wrist_vel = 60.0;

    if (this->wrist->get_position() <= 115.0) {
      this->state = state_e_t::recover;
      this->update();
    }
    break;
  case state_e_t::trans_to_m:
    this->state = state_e_t::recover;
    this->update();
    break;
    this->wrist_vel = 60.0;
    this->arm_target = 155.0;
    this->wrist_target = 191.0;
    if (this->enc_arm->get_deg() < 200.0) {
      this->wrist_target = 40.0;
    }
    if (this->enc_arm->get_deg() < 160.0) {
      this->wrist_target = 0.0;
      this->arm_target = 140.0;
    }

    break;
  }
}

// performs motions
void Arm::execute() {
  update();

  if (wrist_target.has_value()) {
    this->wrist->move_absolute(wrist_target.value(), this->wrist_vel);
  }

  if (arm_target.has_value()) {
    // control loop
    auto err = arm_target.value() - this->enc_arm->get_deg();
    auto output =
        -this->arm_ctrl.update(arm_target.value() - this->enc_arm->get_deg());
    if (output > 3500.0) {
      output = 3500.0;
    }
    this->intake->move_voltage(output);
  } else {
    this->arm_ctrl.reset();
  }
}

void Arm::move_intake(int v) {
  if (!arm_target.has_value()) {
    this->intake->move(v);
  }
}