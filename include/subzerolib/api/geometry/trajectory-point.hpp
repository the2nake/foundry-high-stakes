#pragma once

#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/geometry/pose.hpp"
#include "subzerolib/api/geometry/spline-point.hpp"

#include <string>

struct trajectory_point_s : public point_s {
  trajectory_point_s(double i_t = 0,
                     double i_s = 0,
                     double i_x = 0,
                     double i_vx = 0,
                     double i_y = 0,
                     double i_vy = 0,
                     double i_h = 0,
                     double i_vh = 0)
      : point_s(i_x, i_y), t(i_t), s(i_s), vx(i_vx), vy(i_vy), h(i_h),
        vh(i_vh) {}

  // constructs, but will be missing t, h, and vh
  trajectory_point_s(const spline_point_s &point)
      : s(point.s), point_s(point.x, point.y), vx(point.vx), vy(point.vy) {}

  // will be missing velocities, t and s
  trajectory_point_s(const pose_s &pose) : point_s(pose.x, pose.y), h(pose.h) {}

  double t = 0;
  double s = 0;
  double vx = 0;
  double vy = 0;
  double h = 0;
  double vh = 0;

  double v() const;

  std::string to_string() const;
};

bool operator==(trajectory_point_s &a, trajectory_point_s &b);

template <>
trajectory_point_s lerp<trajectory_point_s, double>(trajectory_point_s a,
                                                    trajectory_point_s b,
                                                    double t);
