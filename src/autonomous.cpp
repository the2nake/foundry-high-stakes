#include "devices.hpp"
#include "main.h"
#include "subzerolib/api/spline/catmull-rom.hpp"
#include "subzerolib/api/trajectory/motion-profile/linear-motion-profile.hpp"
#include "subzerolib/api/trajectory/motion-profile/trapezoidal-motion-profile.hpp"
#include "subzerolib/api/trajectory/spline-trajectory.hpp"

#include <fstream>

/*
void autonomous() {
  // controller->move_to_pose({0.3, 0.3, 270});
  // controller->move_to_pose({-0.6, 0.5, 315});

  std::shared_ptr<ExitCondition<double>> pass_cond{
      new ExitCondition<double>{{0, 0.06}, 100}
  };

  std::shared_ptr<ExitCondition<double>> end_cond{
      new ExitCondition<double>{{0, 0.02}, 200}
  };

  std::vector<pose_s> ctrl = {
      pose_s{  0.0,  0.0,   0.0},
      pose_s{  0.4,  0.6,  45.0},
      pose_s{ -0.2,  0.6,  60.0},
      pose_s{-0.75, 0.75, -45.0}
  };
  std::shared_ptr<CatmullRomSpline> spline{new CatmullRomSpline{ctrl}};
  spline->pad_velocity({0.5, 0.5}, {-0.25, 0.25});

  std::shared_ptr<TrapezoidalMotionProfile> profile{
      new TrapezoidalMotionProfile{chassis->get_max_vel(), 5}
  };

  auto traj =
      SplineTrajectory::Builder(SplineTrajectory::heading_mode_e::pose, 400)
          .with_spline(spline, ctrl)
          .with_chassis(chassis)
          .with_motion_profile(profile)
          .build();
}*/

void slew(double &output, double &prev, double max_delta) {
  max_delta = std::abs(max_delta);
  if (output - prev > max_delta) {
    output = prev + max_delta;
  }

  if (output - prev < -max_delta) {
    output = prev - max_delta;
  }
}

void slew_inc(double &output, double &prev, double max_rate, double msec) {
  double max_delta = max_rate * msec;
  if (prev * output < 0 || std::abs(output) - std::abs(prev) > max_delta) {
    slew(output, prev, max_delta);
  }
}

void turn_to_angle(void *target_ptr) {
  double target = *(double *)target_ptr;
  delete (double *)target_ptr;

  // ! TUNE
  PIDF pid_ang{0.03, 0.02, 0.004, true}; // old kd 0.004
  double error = std::nan("");

  std::uint32_t timestamp = pros::millis();
  std::uint32_t *prev_ptr = &timestamp;
  double ms_ok = 0.0;
  const double delta = 10.0;

  double prev = 0.0;

  while (std::abs(pid_ang.get_output()) > 2.0 || std::isnan(error) ||
         ms_ok < 100.0) {
    if (std::abs(error) > 2) {
      ms_ok = 0.0;
    } else {
      ms_ok += delta;
    }
    error = shorter_turn(imu->degrees(), target);
    auto output = pid_ang.update(error);

    slew_inc(output, prev, 0.01, delta);
    prev = output;

    chassis->move(0, 0, output);
    pros::Task::delay_until(prev_ptr, delta);
  }
}

void turn_to_angle(double target) { turn_to_angle(new double(target)); }

pros::Task *turn_to_angle_async(double target) {
  return new pros::Task(turn_to_angle, new double(target), "pid turning");
}

void move_distance(void *target_ptr) {
  double target = *(double *)target_ptr;
  delete (double *)target_ptr;

  PIDF pid_lin{3.2, 0.25, 0.45, true};
  PIDF pid_ang{0.042, 0.02, 0.004, true};
  double err_lin = std::nan("");
  double err_ang = std::nan("");

  double initial_angle = imu->degrees();
  double initial_y = enc_y->get_deg();

  const double dist_per_deg = 0.16 / 360.0;

  double ms_ok = 0.0;
  const double delta = 10.0;

  double prev_lin = 0.0;

  std::uint32_t timestamp = pros::millis();
  std::uint32_t *prev_ptr = &timestamp;
  while (
      std::abs(pid_lin.get_output()) + std::abs(pid_ang.get_output()) > 2.0 ||
      std::isnan(err_lin) || std::isnan(err_ang) || ms_ok < 100.0 /* ||
       std::abs(err_ang) > 2*/
  ) {
    if (std::abs(err_lin) > 0.03) {
      ms_ok = 0.0;
    } else {
      ms_ok += delta;
    }
    err_lin = target - (enc_y->get_deg() - initial_y) * dist_per_deg;
    err_ang = shorter_turn(imu->degrees(), initial_angle);

    auto out_lin = pid_lin.update(err_lin);
    slew_inc(out_lin, prev_lin, 0.0001, delta);
    prev_lin = out_lin;

    chassis->move(0, pid_lin.update(err_lin), pid_ang.update(err_ang));
    pros::Task::delay_until(prev_ptr, delta);
  }
}

void move_distance(double target) { move_distance(new double(target)); }

pros::Task *move_distance_async(double distance) {
  return new pros::Task(move_distance, new double(distance), "move distance");
}

void stop_task(pros::Task *&task) {
  if (task != nullptr) {
    task->remove();
    delete task;
    task = nullptr;
  }
}

void auto2() {
  // tuning
  // move_distance(0.6);
  // turn_to_angle(120);
}

int time(std::function<void()> fn) {
  auto start = pros::millis();
  fn();
  return pros::millis() - start;
}

std::shared_ptr<CatmullRomSpline>
padded_spline(std::vector<point_s> points, point_s start_v, point_s end_v) {
  auto spline = std::make_shared<CatmullRomSpline>(points);
  spline->pad_velocity(start_v, end_v);
  return spline;
}

void tuning() {
  pros::delay(2000);
  auto start_ts = pros::millis();
  std::ofstream out("/usd/tuning.txt", std::ios::out);
  uint32_t prev = pros::millis();
  std::uint32_t *ptr = &prev;
  while (pros::millis() - start_ts < 2000) {
    chassis->move_vels({0.25, 0.25});
    char msg[100];
    snprintf(msg,
             100,
             "%d %f %f\n",
             pros::millis(),
             chassis->get_actual_vels()[0],
             chassis->get_actual_vels()[1]);
    if (out.is_open()) {
      out << msg;
    }
    pros::Task::delay_until(ptr, 10);
  }
  out.close();
}

void autonomous() {
  auto start_ts = pros::millis();

  pros::Task *motion = nullptr;

  flipper.set_state(true);
  Ramsete ctrl{2.0,
               0.7,
               odom,
               chassis,
               std::unique_ptr<Condition<double>>{
                   new Condition<double>{{0.0, 0.03}, 100}}};

  odom->set_position(0.0, 0.0);
  odom->set_heading(0.0);

  std::shared_ptr<TankModel> model{new TankModel(1.7, 5.0, 3.0, 0.248, 0.3)};
  std::shared_ptr<LinearMotionProfile> fast_profile{
      new TrapezoidalMotionProfile(1.7, 5.0, 2.0)};
  std::shared_ptr<LinearMotionProfile> slow_profile{
      new TrapezoidalMotionProfile(1.0, 3.0, 2.0)};

  // tuning();
  ctrl.follow(
      SplineTrajectory{
          padded_spline(
              {{0, 0}, {0.0, 0.3}, {-0.6, 0.5}},
              {0.0, 0.5},
              {-0.5, 0.5}
              ),
          fast_profile,
          model
  }
          .get_profile(),
      6000);
  /**/

  /*
  move_distance(1.01);
  pros::delay(100);
  turn_to_angle(315.0);
  arm->move_intake(127);
  move_distance(0.16);
  pros::delay(500);
  flipper.set_state(false);

  move_distance(-0.17);
  turn_to_angle(98);
  move_distance(-0.64);
  turn_to_angle(40.0);
  move_distance(-0.33);
  clamp.set_state(true);
  pros::delay(150);

  // go for the stack
  arm->score();
  pros::delay(800);
  turn_to_angle(90);
  // wait until arm is ready
  move_distance(0.43);
  pros::delay(400);
  arm->move_intake(0);

  turn_to_angle(147);
  arm->move_intake(127);
  // flipper.set_state(true);
  move_distance(1.03);
  pros::delay(300); // get the bottom ring in
  move_distance(-0.6);
  turn_to_angle(340);
  clamp.set_state(false);
  turn_to_angle(73);
  /**/

  arm->move_intake(0);
  clamp.set_state(false);
  subzero::log("[i]: auto finished in %d ms", pros::millis() - start_ts);
}
