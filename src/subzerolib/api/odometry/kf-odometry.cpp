#include "subzerolib/api/odometry/kf-odometry.hpp"

void KFOdometry::set_heading(double heading) {
  ImuOdometry::set_heading(heading);
  auto state = KalmanFilter::get_state();
  auto cov = KalmanFilter::get_covariance();
  state(6) = heading;
  KalmanFilter::initialise(state, cov);
}

void KFOdometry::set_position(double x, double y) {
  ImuOdometry::set_position(x, y);
  auto state = KalmanFilter::get_state();
  auto cov = KalmanFilter::get_covariance();
  state(0) = x;
  state(3) = y;
  KalmanFilter::initialise(state, cov);
}

pose_s KFOdometry::get_pose() {
  Eigen::VectorXd state = KalmanFilter::get_state();
  return pose_s{state(0), state(3), state(6)};
}

pose_s KFOdometry::get_vel() {
  Eigen::VectorXd state = KalmanFilter::get_state();
  return pose_s{state(1), state(4), state(7)};
}

void KFOdometry::update() {
  ImuOdometry::update();
  if (is_enabled()) {
    auto pose = ImuOdometry::get_pose();
    auto vel = ImuOdometry::get_vel();
    auto raw_accel_a = pros::Imu::get_all_devices()[0].get_accel();
    auto raw_accel_b = pros::Imu::get_all_devices()[1].get_accel();
    auto accel = rotate_acw(raw_accel_a.y, raw_accel_a.x, pose.h);
    auto accel2 = rotate_acw(raw_accel_b.y, raw_accel_b.x, pose.h);
    accel = 9.8 * accel / 2.0;
    accel2 = 9.8 * accel2 / 2.0;
    if (std::isnan(accel.x) || std::isnan(accel.y)) {
      accel.x = 0;
      accel.y = 0;
    }
    if (std::isnan(accel2.x) || std::isnan(accel2.y)) {
      accel2.x = 0;
      accel2.y = 0;
    }
    Eigen::Vector<double, 8> meas{
        {vel.x, accel.x, accel2.x, vel.y, accel.y, accel2.y, pose.h, vel.h}
    };
    KalmanFilter::update(meas);
    KalmanFilter::predict();
  }
}

void KFOdometry::set_enabled(bool v) { ImuOdometry::set_enabled(v); }

bool KFOdometry::is_enabled() { return ImuOdometry::is_enabled(); }

std::shared_ptr<KFOdometry> KFOdometry::Builder::build() {
  auto gyro_odom = ImuOdometry::Builder::build();
  auto filter = KalmanFilter::Builder::build();

  if (gyro_odom == nullptr || filter == nullptr) {
    return nullptr;
  }

  if (filter->nu == 0) {
    filter->predict();
  } else {
    Eigen::VectorXd vec;
    vec.resize(filter->nu);
    vec.setZero();
    filter->predict(vec);
  }

  return std::shared_ptr<KFOdometry>{
      new KFOdometry{gyro_odom, filter}
  };
}
