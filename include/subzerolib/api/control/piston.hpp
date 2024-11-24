#pragma once

#include "pros/adi.hpp"
#include <memory>

class Piston {
public:
  Piston(std::unique_ptr<pros::adi::DigitalOut> i_piston,
         bool initial_state = false);

  void set_state(bool i_state);
  void toggle();

  bool get_state();

private:
  bool state = false;
  std::unique_ptr<pros::adi::DigitalOut> piston;
};
