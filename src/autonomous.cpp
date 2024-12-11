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
  PIDF pid_ang{0.021, 0.05, 0.006, true}; // old kd 0.004
  double error = std::nan("");

  std::uint32_t timestamp = pros::millis();
  std::uint32_t *prev_ptr = &timestamp;
  double ms_ok = 0.0;
  const double delta = 10.0;

  double prev = 0.0;

  while (std::abs(pid_ang.get_output()) > 2.0 || std::isnan(error) ||
         ms_ok < 50.0) {
    if (std::abs(error) > 3) {
      ms_ok = 0.0;
    } else {
      ms_ok += delta;
    }
    error = shorter_turn(odom->get_pose().heading(), target);
    auto output = pid_ang.update(error);

    slew_inc(output, prev, 0.01, delta);
    prev = output;

    chassis->move(0, 0, output);
    pros::Task::delay_until(prev_ptr, delta);
  }
  chassis->move(0, 0, 0);
}

void turn_to(double target) { turn_to_angle(new double(target)); }

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

  double initial_angle = odom->get_pose().heading();
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
    err_ang = shorter_turn(odom->get_pose().heading(), initial_angle);

    auto out_lin = pid_lin.update(err_lin);
    slew_inc(out_lin, prev_lin, 0.0001, delta);
    prev_lin = out_lin;

    chassis->move(0, pid_lin.update(err_lin), pid_ang.update(err_ang));
    pros::Task::delay_until(prev_ptr, delta);
  }
  chassis->move(0, 0, 0);
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

void local_awp(std::shared_ptr<TankPID> &tank_pid,
               std::shared_ptr<Boomerang> &boom) {
  odom->set_position(-0.6, -1.5);
  odom->set_heading(90.0);
  clamp.set_state(false);
  doinker.set_state(false);
  lifter.set_state(true);

  arm->intake(12000);
  tank_pid->move_to_pose({-0.13, -1.27}, 900);
  lifter.set_state(false);
  pros::delay(400);

  move_distance(-0.09);
  pros::delay(400);
  turn_to(315.0);
  boom->move_to_pose({0.0, -1.52, -0.5}, 1250);
  arm->intake(0);
  arm->score_alliance();
  pros::delay(800);

  tank_pid->move_to_pose({-1.2, -0.6}, 600);
  arm->intake(12000);
  boom->move_to_pose({-1.3, -0.6, -90.0}, 1500);

  boom->move_to_pose({-0.43, -0.6, -90.0}, 700);
  clamp.set_state(true);
  pros::delay(100);
  boom->move_to_pose({-0.6, -1.2}, 2000);
  arm->intake(0.0);
  arm->score();
  while (arm->get_state() != Arm::state_e_t::ready_m) { pros::delay(50); }
}

void worlds_awp1_blue(std::shared_ptr<TankPID> tank_pid,
                      std::shared_ptr<Boomerang> boom,
                      std::shared_ptr<PurePursuit> pp) {
  std::shared_ptr<TankModel> model{new TankModel(1.7, 5.0, 3.0, 0.248, 0.3)};
  std::shared_ptr<LinearMotionProfile> fast_profile{
      new TrapezoidalMotionProfile(1.6, 4.0, 2.0)};
  std::shared_ptr<LinearMotionProfile> slow_profile{
      new TrapezoidalMotionProfile(1.0, 3.0, 2.0)};

  odom->set_position(-0.6, -1.5);
  odom->set_heading(0.0);
  clamp.set_state(false);
  doinker.set_state(false);
  lifter.set_state(false);

  // intake ring under stack
  arm->intake(12000);
  boom->move_to_pose({-1.28, -0.63, -60.0}, 1300);

  // get the goal and score 2 rings
  pros::Task ign{
      [&boom]() { boom->move_to_pose({-0.685, -0.635, -90.0}, 820); }};
  pros::delay(760);
  clamp.set_state(true);
  pros::delay(150);
  boom->move_to_pose({-0.6, -1.35, -20.0}, 1000);
  arm->intake(0.0);
  arm->score();
  pros::delay(800);
  turn_to(45.0);
  clamp.set_state(false);

  // knock
  boom->move_to_pose({-0.3, -1.435, 105.0}, 800);
  doinker.set_state(true);
  pros::delay(150);
  turn_to(20.0);
  doinker.set_state(false);
  //   while (arm->get_state() != Arm::state_e_t::ready_m) { pros::delay(50); }

  // get ring
  arm->intake(12000);
  boom->move_to_pose({0.05, -1.0, 80.0}, 1000);

  // retreat
  move_distance(-0.1);
  turn_to(0.0);
  boom->move_to_pose({0.08, -1.68, -60}, 1200);
  boom->move_to_pose({0.0, -1.2, 0.0}, 300);
  boom->move_to_pose({0.0, -1.69, 0.0}, 800);
  arm->score_alliance();
  pros::delay(700);

  // go to ring (intake is still on)
  boom->move_to_pose({1.15, -1.0, 45.0}, 700);
  boom->move_to_pose({1.1, -0.62, 0.0}, 1500);
  pros::delay(300);
  arm->intake(0);
  turn_to(80.0);
  move_distance(0.03);
  pros::Task ign_2{[&boom]() { boom->move_to_pose({0.6, -0.64, 90.0}, 1300); }};
  pros::delay(600);
  arm->intake(12000);
  pros::delay(650);
  clamp.set_state(true);
  turn_to(0.0);
  arm->score();
  move_distance(0.3);
}

void worlds_awp2_red(std::shared_ptr<TankPID> tank_pid,
                     std::shared_ptr<Boomerang> boom,
                     std::shared_ptr<PurePursuit> pp) {
  odom->set_position(-0.3, -1.5);
  odom->set_heading(180.0);
  clamp.set_state(false);
  doinker.set_state(true);
  lifter.set_state(false);

  // swing top ring off
  move_distance(-0.3);
  turn_to(120.0);

  /*
  // get goal
  boom->move_to_pose({-0.6, -0.6, 150.0}, 1300);
  clamp.set_state(true);
  turn_to_angle(-90.0);
  arm->intake(12000);
  tank_pid->move_to_pose({-1.2, -0.6}, 1000);
  move_distance(-0.05);
  arm->intake(0);
  arm->score();

  // set goal to a "safe" place
  boom->move_to_pose({-0.6, -1.2, 180.0}, 1500);
  clamp.set_state(false);

  // weave around rings
  arm->intake(12000);
  boom->move_to_pose({0.0, -0.9, 90}, 1100);
  boom->move_to_pose({0.6, -1.5, 135}, 1500);
  turn_to_angle(90.0);
  */
}

void autonomous() {
  auto start_ts = pros::millis();

  pros::Task *motion = nullptr;

  Ramsete ramsete{2.1,
                  0.7,
                  odom,
                  chassis,
                  std::unique_ptr<Condition<double>>{
                      new Condition<double>{{0.0, 0.03}, 100}}};

  std::unique_ptr<Condition<double>> boom_cond{
      new Condition<double>{{0.0, 0.05}, 50}
  };
  std::unique_ptr<PIDF> boom_d_pid = std::make_unique<PIDF>(2.7, 0.0, 0.34);
  std::unique_ptr<PIDF> boom_r_pid = std::make_unique<PIDF>(0.33, 0.0, 0.042);
  auto boom = std::make_shared<Boomerang>(chassis,
                                          odom,
                                          std::move(boom_cond),
                                          std::move(boom_d_pid),
                                          std::move(boom_r_pid),
                                          0.3,
                                          0.15);

  std::unique_ptr<Condition<double>> pp_pid_cond{
      new Condition<double>{{0.0, 0.04}, 50}
  };
  std::unique_ptr<PIDF> profiled_lin_pid = std::make_unique<PIDF>(
      0.1, 0.0, 0.0, false, [](double linv) { return linv * 0.9; });
  std::unique_ptr<PIDF> pp_rot_pid = std::make_unique<PIDF>(0.22, 0.0, 0.014);
  auto pp_tank_pid = std::make_shared<TankPID>(chassis,
                                               odom,
                                               std::move(pp_pid_cond),
                                               std::move(profiled_lin_pid),
                                               std::move(pp_rot_pid));

  std::unique_ptr<Condition<double>> pp_cond{
      new Condition<double>{{0.0, 0.04}, 100}
  };
  auto pp =
      std::make_shared<PurePursuit>(pp_tank_pid, odom, std::move(pp_cond), 2);

  std::unique_ptr<Condition<double>> pid_cond{
      new Condition<double>{{0.0, 0.03}, 50}
  };
  std::unique_ptr<PIDF> lin_pid = std::make_unique<PIDF>(3.0, 0.0, 0.0);
  std::unique_ptr<PIDF> rot_pid = std::make_unique<PIDF>(0.28, 0.0, 0.022);
  auto tank_pid = std::make_shared<TankPID>(chassis,
                                            odom,
                                            std::move(pid_cond),
                                            std::move(lin_pid),
                                            std::move(rot_pid));

  std::shared_ptr<TankModel> model{new TankModel(1.7, 5.0, 3.0, 0.248, 0.3)};
  std::shared_ptr<LinearMotionProfile> fast_profile{
      new TrapezoidalMotionProfile(1.6, 5.0, 2.0)};
  std::shared_ptr<LinearMotionProfile> slow_profile{
      new TrapezoidalMotionProfile(1.0, 3.0, 2.0)};

  /*
  // ! not rlly working
  local_awp(tank_pid, boom);
  */
  worlds_awp1_blue(tank_pid, boom, pp);
  // worlds_awp2_red(tank_pid, boom, pp);

  auto trajectory =
      SplineTrajectory{
          padded_spline(
              {{0, 0}, {0.0, 0.3}, {-0.6, 0.5}},
              {0.0, 0.5},
              {-0.5, 0.5}
              ),
          fast_profile,
          model
  }
          .get_trajectory();
  /**/

  // tuning();
  // ramsete.follow(trajectory, 6000);
  // boom->move_to_pose({-0.5, 0.7, 0.0}, 3000);
  // tank_pid->move_to_pose({-0.5, 0.7});
  // pp.follow(trajectory, 0.07, 5000);

  // TODO: profile linear motion

  arm->intake(0);
  clamp.set_state(false);
  subzero::log("[i]: auto finished in %d ms", pros::millis() - start_ts);
}