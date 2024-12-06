#pragma once

#include "pros/rtos.hpp"
#include <functional>

template <typename Input> class AutoUpdater {
public:
  /// @brief create an object for automatic updating
  /// @param iupdater a callable that takes a single argument. usually a lambda
  /// @param igetter a callable that returns a single argument. usually a lambda
  /// @returns the auto updater
  AutoUpdater(std::function<void(Input)> iupdater, std::function<Input()> igetter)
      : updater(std::move(iupdater)), getter(igetter) {}

  /// @brief destroys the auto updater, stopping running updates in the process
  ~AutoUpdater() { stop(); }

  /// @brief begin updating
  /// @param interval milliseconds between updates
  void start(const int interval) {
    update_task = new pros::Task{[this, interval] {
      while (true) {
        uint32_t start = pros::millis();
        const auto ptr = &start;
        this->updater(this->getter());
        pros::Task::delay_until(ptr, interval);
      }
    }};
  }

  /// @brief stops updates
  void stop() {
    if (update_task != nullptr) {
      update_task->remove();
      delete update_task;
      update_task = nullptr;
    }
  }

private:
  std::function<void(Input)> updater = nullptr;
  std::function<Input()> getter = nullptr;

  pros::Task *update_task = nullptr;
};
