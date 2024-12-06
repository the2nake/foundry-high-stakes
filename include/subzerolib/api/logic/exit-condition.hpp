#pragma once

#include "pros/rtos.hpp"
#include <atomic>

// concept kinda sorta stolen from lemlib

template <typename T> class Condition {
public:
  template <typename range_t> struct range_s {
    range_s(range_t imin, range_t imax, bool iinclusive = true)
        : inclusive(iinclusive) {
      if (imin > imax) {
        max = imin;
        min = imax;
      } else {
        min = imin;
        max = imax;
      }
    }
    range_t min;
    range_t max;
    bool inclusive;

    bool contains(range_t val) const {
      if (inclusive) {
        return min <= val && max >= val;
      } else {
        return min < val && max > val;
      }
    }
  };

  /// @brief create an object representing an exit condition
  /// @param irange the range in which to stop
  /// @param imin_ms the minimum duration in milliseconds that the condition
  /// must be true for
  /// @returns the exit condition object
  Condition(range_s<T> irange, int imin_ms = 500)
      : range(irange), min_ms(imin_ms) {
    last_update = pros::millis();
  }

  /// @brief reset the exit condition
  void reset() {
    last_update = pros::millis();
    met_duration = 0;
  }

  /// @brief update the condition with a new value
  /// @param input the value
  void update(T input) {
    uint32_t now = pros::millis();
    if (range.contains(input)) {
      met_duration += now - last_update;
    } else {
      met_duration = 0;
    }
    last_update = now;
  }

  /// @brief check if the condition is met
  /// @returns whether the condition has been true for at least min_ms
  /// milliseconds
  bool is_met() { return met_duration.load() >= min_ms; }

private:
  range_s<T> range;
  int min_ms;
  std::atomic<int> met_duration = 0;
  uint32_t last_update = pros::millis();
};