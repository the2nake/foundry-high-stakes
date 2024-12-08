#include "subzerolib/api/control/pure-pursuit.hpp"
#include "subzerolib/api/geometry/circle.hpp"
#include "subzerolib/api/geometry/pose.hpp"
#include "subzerolib/api/util/logging.hpp"

#include <cassert>
#include <memory>

PurePursuit::PurePursuit(std::shared_ptr<MtpController> ictrl,
                         std::shared_ptr<Odometry> iodom,
                         std::shared_ptr<Condition<double>> ipos_exit,
                         int iresolution)
    : ctrl(std::move(ictrl)), odom(std::move(iodom)),
      pos_exit(std::move(ipos_exit)), resolution(std::max(1, iresolution)) {}

void PurePursuit::follow(const std::vector<trajectory_point_s> &trajectory,
                         double lookahead,
                         int timeout_ms) {

  if (!this->is_settled()) {
    subzero::error("[e]: pure pursuit: currently following another path");
    return;
  }

  if (trajectory.size() < 2) {
    subzero::error("[e]: pure pursuit: too few waypoints (<2)");
    return;
  }

  mutex.take(5);
  settled = false;
  mutex.give();

  lookahead = std::abs(lookahead);
  auto current = trajectory.begin();
  trajectory_point_s carrot = trajectory.front();
  trajectory_point_s goal{trajectory.back()};

  pose_s prev_pose = odom->get_pose();
  pose_s pose = odom->get_pose();

  const uint32_t start = pros::millis();
  std::uint32_t prev_time = start;
  std::uint32_t *prev_ptr = &prev_time;

  pos_exit->reset();

  for (uint32_t duration = 0; duration < timeout_ms;
       duration = pros::millis() - start) {
    pose = odom->get_pose();

    pos_exit->update(pose.dist(goal));
    if (settled || pos_exit->is_met()) {
      break;
    }

    for (int i = 0; i < resolution; ++i) {
      circle_s seek{lerp(prev_pose, pose, (double)i / resolution), lookahead};
      for (; current != trajectory.end() - 2 && seek.contains(*(current + 1));
           ++current)
        ;
    }

    assert(current != trajectory.end() - 1);

    select_carrot(current, lookahead, carrot);

    ctrl->approach_pose(carrot, carrot.v());

    prev_pose = pose;
    pros::Task::delay_until(prev_ptr, 10);
  }

  this->ctrl->stop();

  this->mutex.take(5);
  this->settled = true;
  this->mutex.give();

  subzero::log("[i]: pursuit: (%.02f, %.02f) @ %.0f done in %d ms",
               goal.x,
               goal.y,
               goal.h,
               pros::millis() - start);
}

void PurePursuit::select_carrot(
    std::vector<trajectory_point_s>::const_iterator current,
    double lookahead,
    trajectory_point_s &carrot) {
  segment_s seg{*current, *(current + 1)};
  circle_s seek{this->odom->get_pose(), lookahead};
  auto intersections = seek.intersections(seg);

  if (intersections.size() == 0) {
    carrot = *current;
  } else if (seek.contains(seg.end)) {
    carrot = *(current + 1);
  } else {
    double dist = std::min(intersections.back().dist(seg.end),
                           intersections[0].dist(seg.end));
    carrot = lerp(*current, *(current + 1), dist / seg.length());
  }
}

void PurePursuit::stop() {
  this->mutex.take(5);
  this->settled = true;
  this->mutex.give();
}

void PurePursuit::follow(const std::vector<pose_s> &waypoints,
                         double lookahead,
                         int timeout_ms) {
  std::vector<trajectory_point_s> trajectory;
  trajectory.reserve(waypoints.size());
  for (auto &pose : waypoints) {
    double s = 0.0;
    if (trajectory.size() > 0) {
      s = trajectory.back().s + pose.dist(trajectory.back());
    }
    trajectory.emplace_back(0.0,
                            s,
                            pose.x,
                            std::nan(""),
                            pose.y,
                            std::nan(""),
                            pose.h,
                            std::nan(""));
  }
  this->follow(trajectory, lookahead, timeout_ms);
}

void PurePursuit::follow_async(const std::vector<pose_s> &waypoints,
                               double lookahead,
                               int timeout_ms) {
  pros::Task task{
      [=, this]() { this->follow(waypoints, lookahead, timeout_ms); },
      "pure pursuit"};
}

void PurePursuit::follow_async(
    const std::vector<trajectory_point_s> &trajectory,
    double lookahead,
    int timeout_ms) {

  pros::Task task{
      [=, this]() { this->follow(trajectory, lookahead, timeout_ms); },
      "pure pursuit"};
}