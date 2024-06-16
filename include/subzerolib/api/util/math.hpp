#pragma once

#include <cmath>

const double K_PI = 3.141592654;

double in_rad(double deg);
double in_deg(double rad);

/// @brief finds the shortest turn from h0 to hf.
/// @param h0 initial angle
/// @param hf final angle
/// @param circle_size number of units in a circle. degrees default (360.0)
/// @returns the difference in angle, with (+) values clockwise.
template <typename T>
auto shorter_turn(T h0, T hf, T circle_size = 360.0) -> decltype(hf - h0) {
  circle_size = std::abs(circle_size);
  auto cw = std::fmod(hf - h0, circle_size);
  if (std::abs(cw) < circle_size / 2.0) {
    return cw;
  } else {
    return cw - circle_size;
  }
}