#include "subzerolib/api/util/controls.hpp"
#include "subzerolib/api/util/math.hpp"
#include <algorithm>
double control_components_s::sum() { return lin + rot; }
double control_components_s::sum(double rot_pref) {
  clamp_val<double>(rot_pref, 0.0, 1.0);
  return (1 - rot_pref) * lin + rot_pref * rot;
}

void balance_vels(std::vector<control_components_s> &vels, double max_v,
                  double rot_pref) {
  if (std::abs(std::max_element(
                   vels.begin(), vels.end(),
                   [](control_components_s a, control_components_s b) -> bool {
                     return std::abs(a.sum()) < std::abs(b.sum());
                   })
                   ->sum()) < max_v) {
    return;
  }

  clamp_val<double>(rot_pref, 0.0, 1.0);

  double max_sum = 0.0;
  for (auto &vel : vels) {
    vel.lin *= 1 - rot_pref;
    vel.rot *= rot_pref;
    if (max_sum < std::abs(vel.sum())) {
      max_sum = std::abs(vel.sum());
    }
  }

  for (auto &vel : vels) {
    vel.lin *= max_v / max_sum;
    vel.rot *= max_v / max_sum;
  }
}
