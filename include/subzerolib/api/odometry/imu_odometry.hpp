#pragma once

#include "pros/rtos.hpp"
#include "subzerolib/api/odometry/odometry.hpp"
#include "subzerolib/api/sensors/abstract_encoder.hpp"
#include "subzerolib/api/sensors/abstract_gyro.hpp"

#include <memory>

class ImuOdometry : public Odometry {
public:
  void set_heading(double ih) override {
    lock();
    this->pose.h = ih;
    unlock();
  }
  void set_position(double ix, double iy) override {
    lock();
    this->pose.x = ix;
    this->pose.y = iy;
    unlock();
  }

  pose_s get_pose() override {
    lock();
    auto temp = pose;
    unlock();
    return temp;
  }
  point_s get_vel() override;

  void update() override;
  void set_enabled(bool v) override { enabled = v; }

private:
  pros::Mutex state_mutex;
  bool enabled = true;

  AbstractGyro *gyro;
  std::vector<std::pair<AbstractEncoder *, encoder_conf_s>> x_encs;
  std::vector<std::pair<AbstractEncoder *, encoder_conf_s>> y_encs;

  uint32_t prev_timestamp = 0;
  double prev_heading = 0.0;
  std::vector<double> prev_x_enc_vals;
  std::vector<double> prev_y_enc_vals;

  pose_s pose;

  ImuOdometry() {}
  void lock() {
    while (!this->state_mutex.take(5)) {
      pros::delay(1);
    }
  }
  void unlock() { this->state_mutex.give(); }

public:
  class ImuOdometryBuilder {
  public:
    ImuOdometryBuilder &with_gyro(AbstractGyro *igyro);
    ImuOdometryBuilder &with_x_enc(AbstractEncoder *encoder,
                                   encoder_conf_s conf);
    ImuOdometryBuilder &with_y_enc(AbstractEncoder *encoder,
                                   encoder_conf_s conf);

    std::shared_ptr<ImuOdometry> build();

  private:
    AbstractGyro *gyro;
    std::vector<std::pair<AbstractEncoder *, encoder_conf_s>> x_encs;
    std::vector<std::pair<AbstractEncoder *, encoder_conf_s>> y_encs;
  };
};