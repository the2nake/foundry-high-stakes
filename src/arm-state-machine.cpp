#include "devices.hpp"
#include "pros/screen.hpp"
#include "subzerolib/api/logic/state-machine.ipp"
#include <string>

bool motion_complete(std::unique_ptr<pros::AbstractMotor> &mtr,
                     double pos,
                     double thres = 3.0) { // default value
  return std::abs(mtr->get_position() - pos) < std::abs(thres);
}

namespace arm {
std::atomic<arm_signal_e> signal = arm_signal_e::none;

const double k_thres = 3.0;
const int LIFT_VEL = 200;
const int WRIST_VEL = 70;

const double ACCEPT_LIFT_POS = 460.0;
const double ACCEPT_WRIST_POS = -130.0;

const double READY_LIFT_POS = ACCEPT_LIFT_POS;
const double READY_WRIST_POS = -50.0;

const double SCORE_LIFT_POS = 340.0;
const double SCORE_WRIST_POS = 100.0;

const double RELEASE_LIFT_POS = 650.0;
const double RELEASE_WRIST_POS = 120.0;

bool ready(double lift_pos,
           double wrist_pos,
           double lift_thres = k_thres,
           double wrist_thres = k_thres) {
  return motion_complete(mtr_h_lift, lift_pos, lift_thres) &&
         motion_complete(mtr_wrist, wrist_pos, wrist_thres);
}
arm_state_e state = arm_state_e::recovering;

arm_state_e get_state() { return state; }

void update() {
  if (state == arm_state_e::recovering) {
    if (ready(ACCEPT_LIFT_POS, ACCEPT_WRIST_POS)) {
      state = arm_state_e::accepting;
    }
  }
  if (state == arm_state_e::accepting) {
    if (signal.load() == arm_signal_e::score) {
      state = arm_state_e::ready;
    }
  }
  if (state == arm_state_e::ready) {
    if (ready(READY_LIFT_POS, READY_WRIST_POS, 50, 30)) {
      state = arm_state_e::scoring;
    }
  }
  if (state == arm_state_e::scoring) {
    if (signal.load() == arm_signal_e::recover) {
      state = arm_state_e::recovering;
    } else if (ready(SCORE_LIFT_POS, SCORE_WRIST_POS, 6)) {
      state = arm_state_e::releasing;
    }
  }
  if (state == arm_state_e::releasing) {
    if (ready(RELEASE_LIFT_POS, SCORE_WRIST_POS, 8, 15) ||
        signal.load() == arm_signal_e::recover) {
      state = arm_state_e::recovering;
    }
  }
}
void move(double lift_pos,
          double lift_vel,
          double wrist_pos,
          double wrist_vel) {
  mtr_h_lift->move_absolute(lift_pos, lift_vel);
  mtr_wrist->move_absolute(wrist_pos, wrist_vel);
}

void act() {       // above equals this
  switch (state) { // no code block, has to have colon
  case arm_state_e::recovering:
    if (mtr_wrist->get_position() <= 0) {
      move(ACCEPT_LIFT_POS, 200, ACCEPT_WRIST_POS, 100);
    } else {
      move(RELEASE_LIFT_POS, 200, ACCEPT_WRIST_POS, 100);
    }
    break;
  case arm_state_e::accepting:
    move(ACCEPT_LIFT_POS, LIFT_VEL, ACCEPT_WRIST_POS, WRIST_VEL);
    break;
  case arm_state_e::ready:
    move(READY_LIFT_POS, LIFT_VEL, READY_WRIST_POS, WRIST_VEL);
    break;
  case arm_state_e::scoring:
    move(SCORE_LIFT_POS, LIFT_VEL, SCORE_WRIST_POS, WRIST_VEL);
    break;
  case arm_state_e::releasing:
    if (mtr_wrist->get_position() <= 590) {
      move(RELEASE_LIFT_POS, LIFT_VEL, SCORE_WRIST_POS, 100);
    } else {
      move(RELEASE_LIFT_POS, LIFT_VEL, RELEASE_WRIST_POS, 100);
    }
    break;
  case arm_state_e::none:
    break;
  }
}
}; // namespace arm

/*std::unique_ptr<StateMachine<arm_state_e>> sm_arm{
    StateMachine<arm_state_e>::Builder().with_init(arm_state_e::ready).build()};


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