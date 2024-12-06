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
