#include "subzerolib/api/control/pure-pursuit.hpp"
#include "subzerolib/api/geometry/circle.hpp"
#include "subzerolib/api/geometry/pose.hpp"
#include "subzerolib/api/util/logging.hpp"

#include <cassert>
#include <memory>

PurePursuitController::PurePursuitController(
    std::shared_ptr<MtpController> ichassis,
    std::shared_ptr<Odometry> iodom,
    std::shared_ptr<Condition<double>> ipos_exit,
    int iresolution)
    : controller(std::move(ichassis)), odom(std::move(iodom)),
      pos_exit(std::move(ipos_exit)), resolution(std::max(1, iresolution)) {}

void PurePursuitController::follow(const std::vector<pose_s> &waypoints,
                                   double lookahead,
                                   int ms_timeout) {
  if (!this->is_settled()) {
    subzero::error("[e]: pure pursuit: currently following another path");
    return;
  }

  if (waypoints.size() < 2) {
    subzero::error("[e]: pure pursuit: too few waypoints (<2)");
    return;
  }

  mutex.take(5);
  settled = false;
  mutex.give();

  lookahead = std::abs(lookahead);
  auto current = waypoints.begin();
  pose_s carrot = waypoints.front();
  pose_s goal{waypoints.back()};

  pose_s prev_pose = odom->get_pose();
  pose_s pose = odom->get_pose();

  uint32_t start = pros::millis();
  std::uint32_t prev_time = start;
  std::uint32_t *prev_ptr = &prev_time;

  pos_exit->reset();

  for (uint32_t duration = 0; duration < ms_timeout;
       duration = pros::millis() - start) {
    pose = odom->get_pose();

    pos_exit->update(pose.dist(goal));
    if (settled || pos_exit->is_met()) {
      break;
    }

    for (int i = 0; i < resolution; ++i) {
      circle_s seek{lerp(prev_pose, pose, (double)i / resolution), lookahead};
      for (; current != waypoints.end() - 2 && seek.contains(*(current + 1));
           ++current)
        ;
    }

    assert(current != waypoints.end() - 1);

    select_carrot(current, lookahead, carrot);

    controller->approach_pose(carrot);

    prev_pose = pose;
    pros::Task::delay_until(prev_ptr, 10);
  }

  this->controller->brake();

  this->mutex.take(5);
  this->settled = true;
  this->mutex.give();

  subzero::log("[i]: pure pursuit to (%.02f, %.02f) @ %.0f done",
               goal.x,
               goal.y,
               goal.h);
}

void PurePursuitController::select_carrot(
    std::vector<pose_s>::const_iterator current,
    double lookahead,
    pose_s &carrot) {
  segment_s seg{*current, *(current + 1)};
  circle_s seek{this->odom->get_pose(), lookahead};
  auto intersections = seek.intersections(seg);
  if (intersections.size() == 0) {
    carrot = *current;
  } else {
    double dist = std::min(intersections.back().dist(seg.end),
                           intersections[0].dist(seg.end));
    carrot = lerp(seg.start, seg.end, dist / seg.length());
  }
}

void PurePursuitController::stop() {
  this->mutex.take(5);
  this->settled = true;
  this->mutex.give();
}
