#pragma once

#include "subzerolib/api/chassis/tank-chassis.hpp"
#include "subzerolib/api/control/piston.hpp"
#include "subzerolib/api/logic/state-machine.hpp"
#include "subzerolib/api/odometry/odometry.hpp"
#include "subzerolib/api/sensors/abstract-encoder.hpp"
#include "subzerolib/api/sensors/abstract-gyro.hpp"

#include "pros/motor_group.hpp"
#include <atomic>
#include <memory>

extern std::unique_ptr<pros::MotorGroup> mtr_l;
extern std::unique_ptr<pros::MotorGroup> mtr_r;
extern std::shared_ptr<TankChassis> chassis;

extern std::unique_ptr<pros::AbstractMotor> mtr_h_lift;
extern std::unique_ptr<pros::AbstractMotor> mtr_h_intake;
extern std::unique_ptr<pros::AbstractMotor> mtr_wrist;

extern Piston p_clamp;

/*
extern std::shared_ptr<AbstractGyro> imu_1;
extern std::shared_ptr<AbstractGyro> imu_2;
extern std::shared_ptr<AbstractGyro> imu1;
extern std::shared_ptr<AbstractGyro> imu2;
extern std::shared_ptr<AbstractGyro> mean_imu;

extern std::shared_ptr<AbstractEncoder> enc_x;
extern std::shared_ptr<AbstractEncoder> enc_y;
extern std::shared_ptr<Odometry> odom;
*/

void initialise_devices();

enum class arm_state_e {
  none,
  recovering,
  accepting,
  ready,
  scoring,
  releasing
};

enum class arm_signal_e { none, score, recover };

extern std::unique_ptr<StateMachine<arm_state_e>> sm_arm;

namespace arm {
extern std::atomic<arm_signal_e> signal;

extern arm_state_e state;
arm_state_e get_state();
void update();
void move();
} // namespace arm
