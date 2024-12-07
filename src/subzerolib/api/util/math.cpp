#include "subzerolib/api/util/math.hpp"

double in_rad(double deg) { return deg * K_PI / 180.0; }

double in_deg(double rad) { return 180.0 * rad / K_PI; }

double mod(double x, double modulo) {
  if (modulo == 0) {
    return x;
  }

  while (x < 0) {
    x += modulo;
  }

  while (x >= modulo) {
    x -= modulo;
  }

  return x;
}

point_s closest_point_on_segment(segment_s segment, point_s to_point) {
  point_s relative_point = to_point - segment.start;
  point_s relative_end = segment.end - segment.start;
  auto proj = relative_point.proj_onto(relative_end);

  // clamp the projection
  clamp_val(proj.x, 0.0, relative_end.x);
  clamp_val(proj.y, 0.0, relative_end.y);

  return segment.start + proj;
}
