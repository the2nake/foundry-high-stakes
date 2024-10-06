#include "main.h"
#include "devices.hpp"
#include "ports.h"

#include "subzerolib/api.hpp"

#include <memory>

// #define DEBUG

namespace saturnine {
bool running = true;
};

// TODO: refactor, add "graph_module"?
void odom_disp_loop(void *ignore) {
  std::vector<point_s> past_points;
  const int graphx1 = 330;
  const int graphy1 = 30;
  const int graphx2 = 450;
  const int graphy2 = 150;
  const int graphxmid = (graphx1 + graphx2) / 2.0;
  const int graphymid = (graphy1 + graphy2) / 2.0;
  const int graphw = std::abs(graphx1 - graphx2) / 2.0;
  const int graphh = std::abs(graphy1 - graphy2) / 2.0;

  const double max_x = 0.5;
  const double max_y = 0.5;
  const double padding = 12;

  while (saturnine::running) {
    /*
    auto pose = odom->get_pose();
    past_points.push_back(pose);
    if (past_points.size() > 200) {
      past_points.erase(past_points.begin());
    }
    */

    // graph stuff
    /*
    pros::screen::set_pen(pros::Color::black);
    pros::screen::fill_rect(graphx1 - padding,
                            graphy1 - padding,
                            graphx2 + padding,
                            graphy2 + padding);
    pros::screen::set_pen(pros::Color::gray);
    pros::screen::draw_line(graphx1, graphymid, graphx2,
                            graphymid); // x axis
    pros::screen::draw_line(graphxmid, graphy1, graphxmid,
                            graphy2); // y axis
    pros::screen::set_pen(pros::Color::light_gray);
    pros::screen::draw_rect(graphx1 - padding,
                            graphy1 - padding,
                            graphx2 + padding,
                            graphy2 + padding); // frame

    for (int i = 0; i < past_points.size(); ++i) {
      if (i >= past_points.size() * 0.75) {
        pros::screen::set_pen(pros::Color::white);
      } else if (i >= past_points.size() * 0.5) {
        pros::screen::set_pen(pros::Color::light_gray);
      } else if (i >= past_points.size() * 0.25) {
        pros::screen::set_pen(pros::Color::dark_gray);
      } else {
        pros::screen::set_pen(pros::Color::gray);
      }
      point_s pixel_point{graphxmid + past_points[i].x * graphw / (2.0 * max_x),
                          graphymid -
                              past_points[i].y * graphh / (max_y * 2.0)};

      if (pixel_point.x > graphx2 || pixel_point.x < graphx1 ||
          pixel_point.y > graphy2 || pixel_point.y < graphy1) {
        continue;
      }

      if (i == past_points.size() - 1) {
        pros::screen::draw_circle(pixel_point.x, pixel_point.y, 1);
        pros::screen::draw_pixel(pixel_point.x, pixel_point.y + 3);
        pros::screen::draw_pixel(pixel_point.x, pixel_point.y - 3);
        pros::screen::draw_pixel(pixel_point.x + 3, pixel_point.y);
        pros::screen::draw_pixel(pixel_point.x - 3, pixel_point.y);
        pros::screen::set_pen(pros::Color::black);
        pros::screen::draw_pixel(pixel_point.x, pixel_point.y);
      } else {
        pros::screen::draw_pixel(pixel_point.x, pixel_point.y);
      }
    }
    */
    /*
    pros::screen::set_pen(pros::Color::white);
    subzero::print(0, "filtered pos + vel");
    subzero::print(
        1, "(%6.3f, %6.3f) h: %5.0f", pose.x, pose.y, pose.heading());
    pose = odom->get_vel();
    subzero::print(
        2, "(%6.3f, %6.3f) h: %5.0f", pose.x, pose.y, pose.heading());
    subzero::print(3, "raw pos + vel");
    auto obj = dynamic_cast<KFOdometry *>(odom.get());
    pose = obj->get_raw_pose();
    subzero::print(
        4, "(%6.3f, %6.3f) h: %5.0f", pose.x, pose.y, pose.heading());
    pose = obj->get_raw_vel();
    subzero::print(
        5, "(%6.3f, %6.3f) h: %5.0f", pose.x, pose.y, pose.heading());
    */
    // Eigen::VectorXd d = obj->get_covariance().diagonal();
    // subzero::print(6, "%f %f", d(0), d(1));
    // subzero::print(7, "%f %f", d(2), d(3));
    // subzero::print(8, "%f %f", d(4), d(5));

    pros::delay(10);
  }
}

/*
void score_ring() {
  bool wrist_ready = false;
  bool lift_ready = false;
  switch (arm_pos) {
  case arm_pos_e::ready:
    lift_ready = mtr_close_to(mtr_h_lift, k_lift_top_angle);
    wrist_ready = mtr_close_to(mtr_wrist, k_wrist_ready_angle);
    if (!wrist_ready || !lift_ready) {
      return;
    }
    scoring = true;
    arm_pos = arm_pos_e::mid;
    break;
  case arm_pos_e::mid:
    lift_ready = mtr_close_to(mtr_h_lift, k_lift_mid_angle);
    wrist_ready = mtr_close_to(mtr_wrist, k_wrist_mid_angle);
    if (!wrist_ready || !lift_ready) {
      return;
    }
    scoring = true;
    arm_pos = arm_pos_e::score;
    break;
  case arm_pos_e::score:
    lift_ready = mtr_close_to(mtr_h_lift, k_lift_top_angle);
    wrist_ready = mtr_close_to(mtr_wrist, k_wrist_score_end_angle);
    if (!wrist_ready || !lift_ready) {
      return;
    }
    scoring = false;
    arm_pos = arm_pos_e::ready;
    break;
  case arm_pos_e::descore: // NOTE: intentionally has no break; statement
  case arm_pos_e::invalid:
    arm_pos = arm_pos_e::ready;
    break;
  }
}

void move_arm() {
  switch (arm_pos) {
  case arm_pos_e::ready:
    mtr_h_lift->move_absolute(k_lift_top_angle, 200);
    // TODO: fix me
    if (mtr_h_lift->get_position() > k_lift_mid_angle) {
      mtr_wrist->move_absolute(k_wrist_ready_angle, 60);
    } else {
      mtr_wrist->move_absolute(k_wrist_mid_angle, 60);
    }
    break;
  case arm_pos_e::mid:
    mtr_h_lift->move_absolute(k_lift_mid_angle, 150);
    mtr_wrist->move_absolute(k_wrist_mid_angle, 20);
    break;
  case arm_pos_e::descore:
    mtr_h_lift->move_absolute(k_lift_bottom_angle, 100);
    mtr_wrist->move_absolute(k_wrist_score_trans_angle, 60);
    break;
  case arm_pos_e::score:
    mtr_h_lift->move_absolute(k_lift_top_angle, 100);
    if (mtr_h_lift->get_position() > k_lift_score_thres) {
      mtr_wrist->move_absolute(k_wrist_score_end_angle, 60);
    } else {
      mtr_wrist->move_absolute(k_wrist_score_trans_angle, 60);
    }
    break;
  case arm_pos_e::invalid:
    arm_pos = arm_pos_e::ready;
    break;
  }
}
*/

struct named_port_s {
  const char *name;
  int port;
};

void disp_vel_row(int line,
                  std::map<int, pros::Motor *> &motors,
                  named_port_s left,
                  named_port_s right) {
  left.port = std::abs(left.port);
  right.port = std::abs(right.port);
  if (motors.contains(left.port) && motors.contains(right.port)) {
    pros::Motor *&mtr_l = motors[left.port];
    pros::Motor *&mtr_r = motors[right.port];
    subzero::print(line,
                   "%s: %.0frpm %.0fC   %s: %.0frpm %f.0C",
                   left.name,
                   mtr_l->get_actual_velocity(),
                   mtr_l->get_temperature(),
                   right.name,
                   mtr_r->get_actual_velocity(),
                   mtr_r->get_temperature());
  }
}

void disp_loop(void *ignore) {
  auto devs = pros::Motor::get_all_devices();
  std::map<int, pros::Motor *> motors;
  for (auto &dev : devs) {
    motors.insert(std::pair<int, pros::Motor *>(dev.get_port(), &dev));
  }

  while (saturnine::running) {
    disp_vel_row(0, motors, {"m_l1", DRIVE_L1_PORT}, {"m_r1", DRIVE_R1_PORT});
    disp_vel_row(1, motors, {"m_l2", DRIVE_L2_PORT}, {"m_r2", DRIVE_R2_PORT});
    disp_vel_row(2, motors, {"m_lt", DRIVE_LT_PORT}, {"m_rt", DRIVE_RT_PORT});

    subzero::print(4, "lift : %.1f", mtr_h_lift->get_position());
    subzero::print(5, "wrist: %.1f", mtr_wrist->get_position());

    pros::delay(33);
  }
}

void arm_exec_loop(void *ignore) {
  std::uint32_t timestamp = pros::millis();
  std::uint32_t *prev_ptr = &timestamp;
  while (saturnine::running) {
    sm_arm->exec_behaviour();
    pros::Task::delay_until(prev_ptr, 10);
  }
}

void initialize() {
  subzero::set_log_area(0, 18, 480, 240);

  initialise_devices();

  // pros::Task graphing_task{odom_disp_loop, nullptr, "odom display task"};
  pros::Task display_exec{disp_loop, nullptr, "info display"};
  pros::Task arm_state_exec{arm_exec_loop, nullptr, "arm motion"};
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
#ifdef DEBUG
  autonomous();
#endif

  pros::Controller master(pros::E_CONTROLLER_MASTER);
  /*
  auto pose = odom->get_pose();
  if (std::isnan(pose.h))
    pose.h = 0.0;
  */

  std::uint32_t prev_update = pros::millis();
  std::uint32_t *prev_update_ptr = &prev_update;

  while (saturnine::running) {
    // TODO: adjustments to increase accuracy along diagonals
    // model as a polar radial percentage of a rounded circle?
    double ctrl_rx = master.get_analog(ANALOG_RIGHT_X) / 127.0;
    double ctrl_ry = master.get_analog(ANALOG_RIGHT_Y) / 127.0;
    double ctrl_lx = master.get_analog(ANALOG_LEFT_X) / 127.0;
    double ctrl_ly = master.get_analog(ANALOG_LEFT_Y) / 127.0;

    // pose = odom->get_pose();
    // if (std::isnan(pose.h))
    //   pose.h = 0.0;
    // auto vec = rotate_acw(ctrl_x, ctrl_y, pose.h);

    chassis->move(0, ctrl_ly, 0.7 * ctrl_rx);

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      mtr_h_intake->move(127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      mtr_h_intake->move(-127);
    } else {
      mtr_h_intake->brake();
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      p_clamp.toggle();
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      // TODO: set ready only if ring is present
      arm::flag_score = true;
    } else if (sm_arm->get_curr_state_data().state != arm_state_e::ready) {
      arm::flag_score = false;
    }

    /*
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      if (odom->is_enabled()) {
        odom->set_enabled(false);
      } else {
        odom->set_enabled(true);
      }
    }*/

    // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
    //   odom->set_heading(0);

    pros::Task::delay_until(prev_update_ptr, 20);
  }

  // garbage collection, good practice
  // free all memory
  //   no need in XChassis, everything is a smart pointer
  //   no need for imu, it's a smart pointer
  //   no need in GyroOdometry, everything is a smart pointer
  // delete pointers
}
