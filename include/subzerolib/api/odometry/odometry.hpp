#pragma once

#include "subzerolib/api/geometry/pose.hpp"

#include "pros/rtos.hpp"

class Odometry {
public:
  /// @brief deconstructs the odometry object and stops any running update task
  virtual ~Odometry() {
    if (update_task != nullptr) {
      stop_updating();
      pros::delay(20);
      delete update_task;
      update_task = nullptr;
    }
  }

  struct encoder_conf_s {
    encoder_conf_s(double ioffset, double itravel_per_deg)
        : offset(ioffset), travel_per_deg(itravel_per_deg) {}
    double offset; // right is +x, up is +y
    double travel_per_deg;
  };

  /// @brief sets the heading of the odometry module
  /// @param heading the desired heading
  virtual void set_heading(double heading) = 0;

  /// @brief sets the position of the odometry module
  /// @param x the desired position's x coordinate
  /// @param y the desired position's y coordinate
  virtual void set_position(double x, double y) = 0;

  /// @brief get the current pose
  /// @returns the pose measurement
  virtual pose_s get_pose() = 0;

  /// @brief get the current velocity
  /// @returns the velocity measurement
  virtual pose_s get_vel() = 0;

  /// @brief trigger an update tick
  virtual void update() = 0;

  /// @brief enable/disable the module
  /// @param bool desired state
  virtual void set_enabled(bool) = 0;

  /// @brief check if the module is enabled
  /// @returns whether odometry is active
  virtual bool is_enabled() = 0;

  /// @brief create a task to automatically update odometry at regular intervals
  /// @param every_ms number of milliseconds between updates
  void auto_update(int every_ms = 10) {
    update_delay = every_ms;
    update_task = new pros::Task([&, this] { this->auto_update_loop(); },
                                 "subzerolib: odometry update");
  }

  /// @brief stops automatic updates
  void stop_updating() {
    if (this->update_task != nullptr) {
      this->update_task->notify();
    }
  }

protected:
  Odometry() {}

private:
  int update_delay = 10;
  pros::Task *update_task = nullptr;
  void auto_update_loop() {
    uint32_t prev_time = pros::millis();
    const auto ptr = &prev_time;
    while (!pros::Task::notify_take(true, 0)) {
      this->update();
      pros::Task::delay_until(ptr, update_delay);
    }
  }
};
