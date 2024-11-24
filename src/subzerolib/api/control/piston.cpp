#include "subzerolib/api/control/piston.hpp"

Piston::Piston(std::unique_ptr<pros::adi::DigitalOut> i_piston,
               bool initial_state)
    : state(initial_state), piston(std::move(i_piston))
  {}

void Piston::set_state(bool i_state) {
  state = i_state;
  piston->set_value(state);
}

void Piston::toggle() { Piston::set_state(!state); }

bool Piston::get_state() { return state; }
