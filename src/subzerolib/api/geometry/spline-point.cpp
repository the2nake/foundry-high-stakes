#include "subzerolib/api/geometry/spline-point.hpp"

double spline_point_s::curvature() const {
  auto denom = this->vx * this->vx + this->vy * this->vy;
  denom *= denom * denom;
  return (this->vx * this->ay - this->vy * this->ax) / std::sqrt(denom);
}