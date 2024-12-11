#pragma once

#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"

#include "subzerolib/api/control/pid.hpp"
#include "subzerolib/api/sensors//abstract-encoder.hpp"

#include <memory>

class Arm {
public:
  enum class state_e_t {
    recover,
    accept,
    ready_m,
    score_m,
    trans_to_w,
    trans_to_m,
    ready_w,
    score_w,
    score_al,
  };

  Arm(std::unique_ptr<pros::AbstractMotor> i_intake,
      std::unique_ptr<pros::AbstractMotor> i_wrist,
      std::unique_ptr<AbstractEncoder> i_enc_arm,
      std::unique_ptr<pros::adi::Potentiometer> i_enc_wrist)
      : mtr_intake(std::move(i_intake)), mtr_wrist(std::move(i_wrist)),
        enc_arm(std::move(i_enc_arm)), enc_wrist(std::move(i_enc_wrist)) {}

  std::unique_ptr<pros::AbstractMotor> &get_intake() { return this->mtr_intake; }
  std::unique_ptr<pros::AbstractMotor> &get_wrist() { return this->mtr_wrist; }
  std::unique_ptr<AbstractEncoder> &get_enc_arm() { return this->enc_arm; }
  std::unique_ptr<pros::adi::Potentiometer> &get_enc_wrist() {
    return this->enc_wrist;
  }

  void switch_mode_to_wall(bool mode);
  void toggle_wall_mode();

  void execute();

  void score();
  void score_alliance();

  state_e_t get_state() { return this->state; }

  /// v is from -12000 to 12000
  void intake(int mv);

private:
  void update();
  std::unique_ptr<pros::AbstractMotor> mtr_intake = nullptr;
  std::unique_ptr<pros::AbstractMotor> mtr_wrist = nullptr;
  std::unique_ptr<AbstractEncoder> enc_arm = nullptr;
  std::unique_ptr<pros::adi::Potentiometer> enc_wrist = nullptr;

  state_e_t state = state_e_t::accept;

  std::optional<double> wrist_target = std::nullopt;
  std::optional<double> arm_target = std::nullopt;
  PIDF arm_ctrl{200.00, 1250.0, 22.0, true}; // output: mV
  PIDF wrist_ctrl{30.00, 0.0, 1.0};          // TODO: tuning

  double prev_intake_mv = 0.0;
  double wrist_vel = 100.0;
};