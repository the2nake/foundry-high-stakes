#pragma once

#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "subzerolib/api/util/math.hpp"
#include <atomic>
#include <cstdint>

struct AbstractGyro {
  /// @brief check if the sensor is ready
  /// @returns whether the sensor has finished calibrating
  virtual bool ready() = 0;

  /// @brief get the heading
  /// @returns the heading measurement
  virtual double degrees() = 0;
};

class AbstractImuGyro : public AbstractGyro, public pros::Imu {
public:
  /// @brief create an object to forward function calls to an inertial sensor
  /// @param iport the port of the sensor
  /// @returns a abstract gyroscope object
  AbstractImuGyro(std::uint8_t iport, double i_scale = 1.0)
      : pros::Imu(iport), scale(i_scale) {
    // set_data_rate(5);
  }

  /// @brief check if the sensor is ready
  /// @returns whether the sensor has finished calibrating
  bool ready() override { return !is_calibrating(); }

  /// @brief get the heading
  /// @returns the heading measurement
  double degrees() override {
    if (is_calibrating()) {
      return 0;
    } else {
      if (mutex.take(5)) {
        output += scale * shorter_turn(prev_heading, get_heading(), 360.0);
        prev_heading = get_heading();
        mutex.give();
        return output;
      } else {
        return get_heading();
      }
    }
  }

  void update() {
    if (mutex.take(5)) {
      output += scale * shorter_turn(prev_heading, get_heading(), 360.0);
      prev_heading = get_heading();
      mutex.give();
    }
  }

private:
  pros::Mutex mutex;
  const double scale;
  double prev_heading = 0.0;
  std::atomic<double> output = 0.0;
};
