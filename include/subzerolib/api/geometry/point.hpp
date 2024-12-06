#pragma once

#include "subzerolib/api/util/lerp.hpp"

#include <cmath>
#include <string>

/// @class point / 2d vector class
struct point_s {
  point_s(double ix = 0, double iy = 0) : x(ix), y(iy) {}
  double x;
  double y;

  void operator=(const point_s &b) {
    x = b.x;
    y = b.y;
  }

  double dist(const point_s &b) const;
  double sqdist(const point_s &b) const;

  /// @brief calculate the dot product
  double dot(const point_s &b) const;

  /// @brief vector projection
  point_s proj_onto(const point_s &b) const;

  std::string to_string() const;
};

point_s operator+(point_s a, point_s b);
point_s operator-(point_s a, point_s b);
point_s operator*(double scale, point_s a);
point_s operator*(point_s a, double scale);
point_s operator/(point_s a, double invscale);

template <> point_s lerp<point_s, double>(point_s a, point_s b, double t);
