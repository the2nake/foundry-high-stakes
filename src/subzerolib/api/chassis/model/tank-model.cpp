#include "subzerolib/api/chassis/model/tank-model.hpp"
#include "subzerolib/api/util/math.hpp"

ModelConstraints TankModel::get_constraints(double curvature) const {
  double factor = 1.0;

  if (std::abs(curvature) > K_EPSILON) {
    double radius = std::abs(1.0 / curvature); // direction doesn't matter
    double outer_radius = radius + 0.5 * track_width;
    double inner_radius = radius - 0.5 * track_width;
    factor = 0.5 * (1.0 + inner_radius / outer_radius);
  }

  factor *= 1.0 / (1.0 + drift * curvature);

  return ModelConstraints{vel * factor, accel * factor, decel * factor};
}