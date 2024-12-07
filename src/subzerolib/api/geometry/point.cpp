#include "subzerolib/api/geometry/point.hpp"
#include "subzerolib/api/util/math.hpp"

std::string point_s::to_string() const {
  return "p(" + std::to_string(x) + ", " + std::to_string(y) + ")";
}

point_s operator+(point_s a, point_s b) { return {a.x + b.x, a.y + b.y}; }
point_s operator-(point_s a, point_s b) { return {a.x - b.x, a.y - b.y}; }

point_s operator*(double scale, point_s a) {
  return {a.x * scale, a.y * scale};
}
point_s operator*(point_s a, double scale) {
  return {a.x * scale, a.y * scale};
}
point_s operator/(point_s a, double invscale) {
  return {a.x / invscale, a.y / invscale};
}

template <> point_s lerp<point_s>(point_s a, point_s b, double t) {
  return (1 - t) * a + t * b;
}

double point_s::dist(const point_s &b) const {
  return std::hypot(x - b.x, y - b.y);
}

double point_s::sqdist(const point_s &b) const {
  return (x - b.x) * (x - b.x) + (y - b.y) * (y - b.y);
}

double point_s::dot(const point_s &b) const {
  return this->x * b.x + this->y * b.y;
}

point_s point_s::proj_onto(const point_s &b) const {
  return b * (this->dot(b) / b.dot(b));
}

double point_s::heading_to(const point_s &b) const {
  return 90 - in_deg(std::atan2(b.y - this->y, b.x - this->x));
}
