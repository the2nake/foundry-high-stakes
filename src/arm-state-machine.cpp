#include "devices.hpp"
#include "subzerolib/api/logic/state-machine.ipp"

bool motion_complete(std::unique_ptr<pros::AbstractMotor> &mtr,
                     double thres = 3.0) {
  return std::abs(mtr->get_position() - mtr->get_target_position()) <
         std::abs(thres);
}

namespace arm {
std::atomic<arm_signal_e> signal = arm_signal_e::none;

const double k_thres = 3.0;
const int LIFT_VEL = 200;
const int WRIST_VEL = 70;

const double ACCEPT_LIFT_POS = 390.0;
const double ACCEPT_WRIST_POS = -130.0;

const double READY_LIFT_POS = ACCEPT_LIFT_POS;
const double READY_WRIST_POS = -50.0;

const double SCORE_LIFT_POS = 320.0;
const double SCORE_WRIST_POS = 130.0;

const double RELEASE_LIFT_POS = 600.0;
const double RELEASE_WRIST_POS = 90.0;

bool ready(double lift_thres = k_thres, double wrist_thres = k_thres) {
  return motion_complete(mtr_h_lift, lift_thres) &&
         motion_complete(mtr_wrist, wrist_thres);
}
arm_state_e state = arm_state_e::none;

arm_state_e get_state() { return state; }

void update() {
  if (state == arm_state_e::recovering) {
    if (ready()) {
      state = arm_state_e::accepting;
    }
  }
  if (state == arm_state_e::accepting) {
    if (signal == arm_signal_e::score) {
      state = arm_state_e::ready;
    }
  }
  if (state == arm_state_e::ready) {
    if (ready(50, 30)) {
      state = arm_state_e::scoring;
    }
  }
  if (state == arm_state_e::scoring) {
    if (signal == arm_signal_e::recover) {
      state = arm_state_e::recovering;
    } else if (ready(6)) {
      state = arm_state_e::releasing;
    }
  }
  if (state == arm_state_e::releasing) {
    if (ready(k_thres, 15) || signal == arm_signal_e::recover) {
      state = arm_state_e::recovering;
    }
  }
}
void move(double lift_pos, double lift_vel, double wrist_pos,double wrist_vel) {
  mtr_h_lift->move_absolute(lift_pos, lift_vel);
  mtr_wrist->move_absolute(wrist_pos,wrist_vel);
}
  
void act() {      // above equals this
  switch (state) { // no code block, has to have colon
  case arm_state_e::recovering:
    break;
  }
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