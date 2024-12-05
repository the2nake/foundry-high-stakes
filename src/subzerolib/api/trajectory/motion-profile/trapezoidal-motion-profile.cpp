#include "subzerolib/api/trajectory/motion-profile/trapezoidal-motion-profile.hpp"

#include "subzerolib/api/trajectory/motion-profile/linear-motion-profile.hpp"

#include <cmath>
#include <iterator>

TrapezoidalMotionProfile::TrapezoidalMotionProfile(double imax_vel,
                                                   double imax_accel,
                                                   double imax_decel)
    : max_vel(std::abs(imax_vel)), max_accel(std::abs(imax_accel)),
      max_decel(imax_decel) {
  if (std::isnan(imax_decel)) {
    imax_decel = -std::abs(imax_accel);
  }
}

void TrapezoidalMotionProfile::set_resolution(double step_length) {
  resolution = std::max(0.001, std::abs(step_length));
}

void TrapezoidalMotionProfile::generate(double distance) {
  points.clear();
  std::vector<profile_point_s> points_from_back;

  // start with distance parameterization
  points.emplace_back(0, 0, 0); // change when rev
  points_from_back.emplace_back(0, distance, 0);

  // NOTE: negate if gone backwards
  while (points_from_back.back().x - points.back().x > resolution) {

    profile_point_s front_head{0, 0, 0};
    front_head.x = points.back().x + resolution; // change when rev
    // kinematics
    front_head.v =
        std::min(max_vel,
                 std::sqrt(points.back().v * points.back().v +
                           2 * max_accel * resolution)); // change when rev
    front_head.t = 2 * resolution / (front_head.v + points.back().v);
    points.push_back(front_head);

    if (points_from_back.back().x - points.back().x < resolution) {
      break;
    }

    profile_point_s back_head{0, 0, 0};
    back_head.x = points_from_back.back().x - resolution;
    back_head.v = std::min(
        max_vel,
        std::sqrt(points_from_back.back().v * points_from_back.back().v +
                  2 * max_decel * resolution));
    points_from_back.back().t =
        2 * resolution / (back_head.v + points_from_back.back().v);
    points_from_back.push_back(back_head);
  }

  if (points_from_back.size() > 0 && points.size() > 0) {
    points_from_back.back().t =
        2 * resolution / (points_from_back.back().v + points.back().v);
  }
  points.insert(points.end(),
                std::make_move_iterator(points_from_back.crbegin()),
                std::make_move_iterator(points_from_back.crend()));
  points_from_back.clear();
  for (int i = 1; i < points.size(); ++i) {
    points[i].t += points[i - 1].t;
  }
}

LinearMotionProfile::profile_point_s
TrapezoidalMotionProfile::get_point_at_time(double time) {
  // change when rev
  // assume as trajectory is sorted by time
  if (points.size() == 0) {
    // TODO: error msg
    return {0, 0, 0};
  }
  for (int i = 0; i < points.size(); ++i) {
    if (points[i].t > time) {
      LinearMotionProfile::profile_point_s start = points[i - 1];
      LinearMotionProfile::profile_point_s end = points[i];
      double factor = (time - start.t) / (end.t - start.t);
      return {time,
              std::lerp(start.x, end.x, factor),
              std::lerp(start.v, end.v, factor)};
    }
  }
  return points.back();
}

LinearMotionProfile::profile_point_s
TrapezoidalMotionProfile::get_point_at_distance(double distance) {
  // change when rev
  // assume as trajectory is sorted by time
  if (points.size() == 0) {
    // TODO: error msg
    return {0, 0, 0};
  }
  for (int i = 0; i < points.size(); ++i) {
    if (points[i].x > distance) {
      LinearMotionProfile::profile_point_s start = points[i - 1];
      LinearMotionProfile::profile_point_s end = points[i];
      double factor = (distance - start.x) / (end.x - start.x);
      return {std::lerp(start.t, end.t, factor),
              distance,
              std::lerp(start.v, end.v, factor)};
    }
  }
  return points.back();
}
