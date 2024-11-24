#pragma once

#include "subzerolib/api/geometry/point.hpp"
#include <cmath>

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
