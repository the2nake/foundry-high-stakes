#pragma once

#include "pros/distance.hpp"
#include "subzerolib/api/chassis/tank-chassis.hpp"
#include "subzerolib/api/control/piston.hpp"
#include "subzerolib/api/logic/state-machine.hpp"
#include "subzerolib/api/odometry/odometry.hpp"
#include "subzerolib/api/sensors/abstract-encoder.hpp"
#include "subzerolib/api/sensors/abstract-gyro.hpp"
#include "subsystems/arm.hpp"

#include "pros/motor_group.hpp"
#include <atomic>
#include <memory>

extern std::shared_ptr<TankChassis> chassis;

// extern std::unique_ptr<pros::AbstractMotor> mtr_intake;
// extern std::unique_ptr<pros::AbstractMotor> mtr_wrist;

extern Piston clamp;
extern Arm arm;

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
