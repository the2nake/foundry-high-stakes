#include "subzerolib/api/control/pure-pursuit.hpp"
#include "subzerolib/api/geometry/circle.hpp"
#include "subzerolib/api/geometry/pose.hpp"
#include "subzerolib/api/util/auto-updater.hpp"
#include "subzerolib/api/util/logging.hpp"

#include <memory>

// TODO: refactor pure pursuit code it's ugly as fuck

PurePursuitController::PurePursuitController(
    std::shared_ptr<MTPController> ichassis,
    std::shared_ptr<Odometry> iodom,
    std::shared_ptr<ExitCondition<double>> ipos_exit_condition)
    : controller(std::move(ichassis)), odom(std::move(iodom)),
      pos_exit_condition(std::move(ipos_exit_condition)) {}

void PurePursuitController::follow(std::vector<pose_s> iwaypoints,
                                   double lookahead,
                                   int ms_timeout,
                                   int iresolution) {
  pos_exit_condition->reset();
  waypoints = iwaypoints;
  resolution = std::max(1, iresolution);
  lookahead = std::abs(lookahead);

  mutex.take(5);
  motion_complete = false;
  mutex.give();

  pose_s carrot = waypoints.front();

  pose_s prev_pose = odom->get_pose();
  pose_s curr_pose = odom->get_pose();
  circle_s seek_circle(curr_pose, lookahead);

  uint32_t start = pros::millis();
  pose_s goal{waypoints.back()};
  pos_exit_condition_updater = std::make_unique<AutoUpdater<double>>(
      [&](double val) { this->pos_exit_condition->update(val); },
      [&, goal]() -> double { return this->odom->get_pose().dist(goal); });
  pos_exit_condition_updater->start(10);

  std::uint32_t prev_time = pros::millis();
  std::uint32_t *prev_ptr = &prev_time;
  for (uint32_t duration = 0; duration < ms_timeout;
       duration = pros::millis() - start) {
    curr_pose = odom->get_pose();

    for (int i = 0; i < resolution; ++i) {
      auto check_pose = lerp(prev_pose, curr_pose, i * 1.0 / resolution);
      seek_circle = circle_s(check_pose, lookahead);
      while (waypoints.size() > 1 && seek_circle.contains(waypoints[1])) {
        waypoints.erase(waypoints.begin());
      }
    }

    select_carrot(curr_pose, lookahead, carrot);

    // use chassis controller implementation
    controller->approach_pose(carrot);
    if (motion_complete || pos_exit_condition->is_met()) {
      break;
    }

    prev_pose = curr_pose;
    pros::Task::delay_until(prev_ptr, 10);
  }

  pos_exit_condition_updater->stop();

  controller->brake();
  mutex.take(5);
  motion_complete = true;
  mutex.give();

  subzero::log("[i]: pure pursuit to (%.02f, %.02f) @ %.0f done",
               goal.x,
               goal.y,
               goal.h);
}

void PurePursuitController::select_carrot(pose_s pose,
                                          double lookahead,
                                          pose_s &carrot) {
  // select carrot pose, heading is lerped if intersection is used
  if (waypoints.size() == 1) {
    carrot = waypoints[0];
  } else {
    segment_s segment{waypoints[0], waypoints[1]};
    circle_s seek_circle = circle_s(pose, lookahead);
    auto intersections = seek_circle.intersections(segment);
    if (intersections.size() == 0) {
      carrot = waypoints[0];
    } else {
      if (carrot.dist(segment.end) > intersections.back().dist(segment.end)) {
        auto prop = intersections.back().dist(segment.start) / segment.length();
        carrot = lerp(waypoints[0], waypoints[1], prop);
        // assert intersections.back() rougheq lerp result
      } else {
        auto prop = intersections[0].dist(segment.start) / segment.length();
        carrot = lerp(waypoints[0], waypoints[1], prop);
        // assert intersections[0] rougheq lerp result
      }
    }
  }
}

void PurePursuitController::stop() { motion_complete = true; }
