#include "devices.hpp"
#include "subzerolib/api/logic/state-machine.ipp"

bool motion_complete(std::unique_ptr<pros::AbstractMotor> &mtr,
                     double thres = 3.0) {
  return std::abs(mtr->get_position() - mtr->get_target_position()) <
         std::abs(thres);
}

namespace arm {
std::atomic<arm_signal_e> arm_signal = arm_signal_e::none;

const double k_thres = 3.0;

bool arm_and_wrist_ready() {
  return motion_complete(mtr_h_lift, k_thres) &&
         motion_complete(mtr_wrist, k_thres);
}

}; // namespace arm

std::unique_ptr<StateMachine<arm_state_e>> sm_arm{
    StateMachine<arm_state_e>::Builder().with_init(arm_state_e::ready).build()};

/*
// TODO: declare the thing somewhere else

using exit_pair = std::pair<arm_state_e, std::function<bool()>>;

namespace arm {

const double k_lift_ready = 420.0;
const double k_lift_carry = 350.0;
const double k_lift_score = 350.0;
const double k_lift_rec = 470.0;

const double k_wrist_ready = -130.0;
const double k_wrist_carry = -60.0;
const double k_wrist_score = 60.0;
const double k_wrist_rec = 90.0;

std::atomic<bool> flag_score = false;

void ready_cb() {
  mtr_h_lift->move_absolute(k_lift_ready, 200);
  mtr_wrist->move_absolute(k_wrist_ready, 100);
}
bool ready_to_carry() { return flag_score.load() && arm_and_wrist_ready(); }
std::map<arm_state_e, std::function<bool()>> ready_exit = {
    exit_pair(arm_state_e::carry, ready_to_carry)};

void carry_cb() {
  mtr_h_lift->move_absolute(k_lift_carry, 200);
  mtr_wrist->move_absolute(k_wrist_carry, 100);
}
std::map<arm_state_e, std::function<bool()>> carry_exit = {
    exit_pair(arm_state_e::score, arm_and_wrist_ready)};

void score_cb() {
  mtr_h_lift->move_absolute(k_lift_score, 200);
  mtr_wrist->move_absolute(k_wrist_score, 100);
}
std::map<arm_state_e, std::function<bool()>> score_exit = {
    exit_pair(arm_state_e::recover, arm_and_wrist_ready)};

void rec_cb() {
  mtr_h_lift->move_absolute(k_lift_rec, 200);
  mtr_wrist->move_absolute(k_wrist_rec, 100);
}
std::map<arm_state_e, std::function<bool()>> rec_exit = {
    exit_pair(arm_state_e::ready, arm_and_wrist_ready)};
}; // namespace arm

std::unique_ptr<StateMachine<arm_state_e>> sm_arm{
    StateMachine<arm_state_e>::Builder()
        .with_state({arm_state_e::ready, arm::ready_cb, arm::ready_exit})
        .with_state({arm_state_e::carry, arm::carry_cb, arm::carry_exit})
        .with_state({arm_state_e::score, arm::score_cb, arm::score_exit})
        .with_state({arm_state_e::recover, arm::rec_cb, arm::rec_exit})
        .with_init(arm_state_e::ready)
        .build()};
        */