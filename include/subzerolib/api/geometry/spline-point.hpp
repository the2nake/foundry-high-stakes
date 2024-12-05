#pragma once

#include "subzerolib/api/geometry/point.hpp"

#include <cmath>

struct spline_point_s : public point_s {
  spline_point_s(double ix = 0,
                 double iy = 0,
                 double is = 0,
                 double ivx = 0,
                 double ivy = 0,
                 double iax = 0,
                 double iay = 0)
      : point_s(ix, iy), s(is), vx(ivx), vy(ivy), ax(iax), ay(iay) {}
  double s; // distance

  double vx;
  double vy;
  double v() const { return std::hypot(vx, vy); }

  double ax;
  double ay;
  double a() const { return std::hypot(ax, ay); }

  // returns the signed curvature
  // positive curvature is acw
  double curvature() const;
};
