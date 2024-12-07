#include "subzerolib/api/control/pure-pursuit.hpp"
#include "subzerolib/api/geometry/circle.hpp"
#include "subzerolib/api/geometry/pose.hpp"
#include "subzerolib/api/util/auto-updater.hpp"
#include "subzerolib/api/util/logging.hpp"

#include <memory>

PurePursuitController::PurePursuitController(
    std::shared_ptr<MtpController> ichassis,
    std::shared_ptr<Odometry> iodom,
    std::shared_ptr<Condition<double>> ipos_exit)
    : controller(std::move(ichassis)), odom(std::move(iodom)),
      pos_exit(std::move(ipos_exit)) {}

void PurePursuitController::follow(const std::vector<pose_s> &iwaypoints,
                                   double lookahead,
                                   int ms_timeout,
                                   int iresolution) {
  if (!this->is_settled()) {
    subzero::error("[e]: ramsete: currently following another trajectory");
    return;
  }

  waypoints = iwaypoints;
  resolution = std::max(1, iresolution);
  lookahead = std::abs(lookahead);

  mutex.take(5);
  settled = false;
  mutex.give();

  pose_s carrot = waypoints.front();

  pose_s prev_pose = odom->get_pose();
  pose_s pose = odom->get_pose();

  uint32_t start = pros::millis();
  pose_s goal{waypoints.back()};

  pos_exit->reset();

  std::uint32_t prev_time = pros::millis();
  std::uint32_t *prev_ptr = &prev_time;

  for (uint32_t duration = 0; duration < ms_timeout;
       duration = pros::millis() - start) {
    pose = odom->get_pose();
    pos_exit->update(pose.dist(goal));

    for (int i = 0; i < resolution; ++i) {
      circle_s seek{lerp(prev_pose, pose, (double)i / resolution), lookahead};
      while (waypoints.size() > 1 && seek.contains(waypoints[1])) {
        waypoints.erase(waypoints.begin());
      }
    }

    select_carrot(pose, lookahead, carrot);

    // use chassis controller implementation
    controller->approach_pose(carrot);
    if (settled || pos_exit->is_met()) {
      break;
    }

    prev_pose = pose;
    pros::Task::delay_until(prev_ptr, 10);
  }

  controller->brake();

  mutex.take(5);
  settled = true;
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

void PurePursuitController::stop() { settled = true; }
