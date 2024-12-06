#include "subzerolib/api/geometry/spline-point.hpp"

double spline_point_s::curvature() const {
  auto denom = this->vx * this->vx + this->vy * this->vy;
  denom *= denom * denom;
  return (this->vx * this->ay - this->vy * this->ax) / std::sqrt(denom);
}

template <>
spline_point_s
lerp<spline_point_s, double>(spline_point_s a, spline_point_s b, double t) {
  return spline_point_s{
      lerp(a.x, b.x, t),
      lerp(a.y, b.y, t),
      lerp(a.s, b.s, t),
      lerp(a.vx, b.vx, t),
      lerp(a.vy, b.vy, t),
      lerp(a.ax, b.ax, t),
      lerp(a.ay, b.ay, t),
  };
}
