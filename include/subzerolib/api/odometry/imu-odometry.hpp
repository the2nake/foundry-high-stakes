#pragma once

#include "subzerolib/api/odometry/odometry.hpp"
#include "subzerolib/api/sensors/abstract-encoder.hpp"
#include "subzerolib/api/sensors/abstract-gyro.hpp"
#include "subzerolib/api/util/logging.hpp"

#include "pros/rtos.hpp"
#include <memory>

class ImuOdometry : public Odometry {
public:
  virtual ~ImuOdometry() {}

  /// @brief sets the heading of the odometry module
  /// @param heading the desired heading
  virtual void set_heading(double i_h) override;

  /// @brief sets the position of the odometry module
  /// @param x the desired position's x coordinate
  /// @param y the desired position's y coordinate
  virtual void set_position(double i_x, double i_y) override;

  /// @brief get the current pose
  /// @returns the pose measurement
  virtual pose_s get_pose() override {
    lock();
    auto temp = pose;
    unlock();
    return temp;
  }

  /// @brief get the current velocity
  /// @returns the velocity measurement
  virtual pose_s get_vel() override {
    lock();
    auto temp = vel;
    unlock();
    return temp;
  }

  /// @brief trigger an update tick
  virtual void update() override;

  /// @brief enable/disable the module
  /// @param bool desired state
  virtual void set_enabled(bool v) override { enabled = v; }

  /// @brief check if the module is enabled
  /// @returns whether odometry is active
  virtual bool is_enabled() override { return enabled.load(); }

private:
  pros::Mutex mutex;
  std::atomic<bool> enabled = true;

  std::shared_ptr<AbstractGyro> gyro;
  std::vector<std::pair<std::shared_ptr<AbstractEncoder>, encoder_conf_s>>
      x_encs;
  std::vector<std::pair<std::shared_ptr<AbstractEncoder>, encoder_conf_s>>
      y_encs;

  uint32_t prev_timestamp = 0;
  double prev_heading = 0.0;
  std::vector<double> prev_x_enc_vals;
  std::vector<double> prev_y_enc_vals;

  pose_s pose{0, 0, 0};
  pose_s vel{0, 0, 0};

  ImuOdometry() {}
  void lock() {
    if (!mutex.take(5)) {
      subzero::error("[e]: gyro odom mutex failed to take");
    }
  }
  void unlock() {
    if (!mutex.give()) {
      subzero::error("[e]: gyro odom mutex failed to return");
    }
  }

  void update_pose_from_filter();

public:
  class Builder {
  public:
    Builder() {}
    Builder(Builder &&other) {
      gyro = other.gyro;
      x_encs = other.x_encs;
      y_encs = other.y_encs;
    }

    Builder &with_gyro(std::shared_ptr<AbstractGyro> igyro);

    Builder &with_x_enc(std::shared_ptr<AbstractEncoder> encoder,
                        encoder_conf_s conf);

    Builder &with_y_enc(std::shared_ptr<AbstractEncoder> encoder,
                        encoder_conf_s conf);

    std::shared_ptr<ImuOdometry> build();

  private:
    std::shared_ptr<AbstractGyro> gyro;
    std::vector<std::pair<std::shared_ptr<AbstractEncoder>, encoder_conf_s>>
        x_encs;
    std::vector<std::pair<std::shared_ptr<AbstractEncoder>, encoder_conf_s>>
        y_encs;
  };

protected:
  ImuOdometry(ImuOdometry &&other) {
    lock();
    // disable the other one
    other.mutex.take();
    enabled = other.enabled.load();
    gyro = std::move(other.gyro);
    x_encs = other.x_encs;
    y_encs = other.y_encs;
    prev_timestamp = other.prev_timestamp;
    prev_heading = other.prev_heading;
    prev_x_enc_vals = other.prev_x_enc_vals;
    prev_y_enc_vals = other.prev_y_enc_vals;
    pose = other.pose;
    unlock();
  }
};