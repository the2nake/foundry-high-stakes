#pragma once

#include "api.h"
#include "subzerolib/api/control/pid.hpp"
#include <atomic>
#include <map>

class Arm {

  enum class target_e { score, accept, recover, none };
  std::map<target_e, double> targets = {
      {  Arm::target_e::score, 260.0},
      { Arm::target_e::accept,   0.0},
      {Arm::target_e::recover,   0.0},
      {   Arm::target_e::none,   0.0},
  };
  double target_pos() { return targets[this->target.load()]; }

public:
  Arm(std::unique_ptr<pros::AbstractMotor> iintake,
      std::unique_ptr<pros::AbstractMotor> iwrist,
      pros::adi::Potentiometer ipot,
      std::unique_ptr<PIDF> ictrl)
      : intake(std::move(iintake)), wrist(std::move(iwrist)), pot(ipot),
        wrist_ctrl(std::move(ictrl)) {}

  void score();
  void move_intake(double volts);
  void override_intake(double volts);
  void stop_intake();
  void execute();

  double wrist_temp() { return this->wrist->get_temperature(); }
  double intake_temp() { return this->intake->get_temperature(); }

private:
  std::unique_ptr<pros::AbstractMotor> intake;
  std::unique_ptr<pros::AbstractMotor> wrist;
  pros::adi::Potentiometer pot;

  std::unique_ptr<PIDF> wrist_ctrl;

  std::atomic<target_e> target = Arm::target_e::accept;
};