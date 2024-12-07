#pragma once

#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/geometry/segment.hpp"
#include <cmath>
#include <vector>

const double K_PI = 3.141592654;
const double K_EPSILON = 0.00001; // range at which things are the same
const double K_SQRT_2 = 1.414213562;

/// @brief return if two values are roughly equal
/// @tparam T the type of the values
/// @param a the first value
/// @param b the second value
/// @returns if the values are roughly the same
template <typename T> bool rougheq(T a, T b) {
  return std::abs(a - b) < K_EPSILON;
}

/// @brief converts from degrees to radians
/// @param deg the value in degrees
/// @returns a value in radians
double in_rad(double deg);

/// @brief converts from radians to degrees
/// @param rad the value in radians
/// @returns a value in degrees
double in_deg(double rad);

/// @brief calculates a solely positive modulus
/// @param a the first argument
/// @param circ the cycle amount
/// @returns 0 <= x < circ
double mod(double a, double circ);

/// @brief finds the shortest turn from h0 to hf.
/// @tparam T the type of the values
/// @param h0 initial angle
/// @param hf final angle
/// @param circle_size number of units in a circle. degrees default (360.0)
/// @returns the difference in angle, with (+) values clockwise.
template <typename T>
auto shorter_turn(T h0, T hf, T circle_size = 360.0) -> decltype(hf - h0) {
  auto rightward_angle = mod(hf - h0, circle_size);
  if (std::abs(rightward_angle) < std::abs(circle_size / 2.0)) {
    return rightward_angle;
  } else {
    return rightward_angle - circle_size;
  }
}

/// @brief clamps a value between two ranges
/// @tparam T the type of the values
/// @param val reference to the value
/// @param min the minimum
/// @param max the maximum
template <typename T> void clamp_val(T &val, T min, T max) {
  if (max < min) {
    std::swap(max, min);
  }
  if (max < val) {
    val = max;
  } else if (min > val) {
    val = min;
  }
}

/// @brief clamps the distance to the origin
/// @tparam T the type of the values
/// @param max_dist the distance maximum
/// @param x reference to the x coordinate
/// @param y reference to the y coordinate
template <typename T> void clamp_distance(T max_dist, T &x, T &y) {
  double d = std::hypot(x, y);
  if (std::abs(d) > max_dist) {
    double sin = y / d;
    double cos = x / d;
    d = std::min(max_dist, d);
    y = sin * d;
    x = cos * d;
  }
}

/// @brief rotate a point anticlockwise
/// @tparam T the type of the values
/// @param x the x coordinate
/// @param y the y coordinate
/// @param deg the angle to rotate in degrees
/// @returns the rotated coordinates
template <typename T> point_s rotate_acw(T x, T y, T deg) {
  double rad = in_rad(-deg);
  return {x * cos(rad) + y * sin(rad), y * cos(rad) - x * sin(rad)};
}

/// @brief find the closest point on a line segment to a point
point_s closest_point_on_segment(segment_s segment, point_s &to_point);

template <typename Point>
Point closest_point_on_segment(Point start, Point end, point_s &to_point) {
  static_assert(std::is_base_of<point_s, Point>::value,
                "Point must be derived from point_s");

  point_s relative_point = to_point - start;
  point_s relative_end = end - start;
  auto proj = relative_point.proj_onto(relative_end);

  // clamp the projection
  clamp_val(proj.x, 0.0, relative_end.x);
  clamp_val(proj.y, 0.0, relative_end.y);

  auto t = start.dist(proj) / start.dist(end);

  return lerp(start, end, t);
}

/// @brief find the closest point on a linear spline to a point
///
/// this function uses an optional heuristic to speed up calculation, when
/// setting optimal to false. this may cause issues in paths with relatively
/// large distances between points
///
/// @param path a vector containing ordered points forming the path
/// @param to_point the point to compare against
/// @param optimal whether to use an optimal algorithm or a fast algorithm
/// @returns the closest point on the path to the point
template <typename Point>
Point closest_point_on_path(const std::vector<Point> &path,
                            point_s to_point,
                            bool optimal = true) {
  static_assert(std::is_base_of<point_s, Point>::value,
                "Point must be derived from point_s");
  if (path.size() < 2) {
    // invalid path
    return {0.0, 0.0};
  }

  if (optimal) {
    Point min_point;
    double min = std::numeric_limits<double>::max();
    for (int i = 0; i < path.size() - 1; ++i) {
      Point point =
          closest_point_on_segment<Point>(path[i], path[i + 1], to_point);
      double sqdistance = to_point.sqdist(point);
      if (sqdistance < min) {
        min = sqdistance;
        min_point = point;
      }
    }
    return min_point;
  } else {
    int min_index = -1;
    auto min = std::numeric_limits<double>::max();
    for (const Point &path_point : path) {
      auto dx = (to_point.x - path_point.x);
      auto dy = (to_point.y - path_point.y);
      if (dx * dx + dy * dy < min) {
        min_index = &path_point - &path[0];
      }
    }

    if (min_index == 0) {
      return closest_point_on_segment<Point>(path[0], path[1], to_point);
    } else if (min_index == path.size() - 1) {
      return closest_point_on_segment<Point>(
          path[path.size() - 2], path[path.size() - 1], to_point);
    } else {
      return std::min(closest_point_on_segment<Point>(
                          path[min_index], path[min_index + 1], to_point),
                      closest_point_on_segment<Point>(
                          path[min_index - 1], path[min_index], to_point),
                      [&to_point](Point a, Point b) -> bool {
                        return to_point.dist(a) < to_point.dist(b);
                      });
    }
  }
}