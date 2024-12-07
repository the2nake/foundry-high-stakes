#include "subzerolib/api/control/mtp-controller.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "subzerolib/api/util/logging.hpp"

void MtpController::move_to_pose(pose_s target) {
  auto initial = pros::competition::get_status();
  auto start = pros::millis();
  std::uint32_t time = start;
  std::uint32_t *ptr = &time;
  while (!this->is_settled() && pros::competition::get_status() == initial) {
    approach_pose(target);
    pros::Task::delay_until(ptr, 10);
  }
  brake();
  subzero::log("[i]: mtp complete: %.2f, %.2f @ %.0f in %d ms",
               target.x,
               target.y,
               target.h,
               pros::millis() - start);
}