#include "subzerolib/api/odometry/imu_odometry.hpp"
#include "subzerolib/api/util/math.hpp"

void ImuOdometry::update() {
  double dh = shorter_turn(prev_heading, gyro->heading());
  if (std::isnan(dh)) {
    dh = 0;
  }
  // for reference: maximum rotation = 2 deg / 10 ms
  bool is_low_turn = std::abs(dh) < 0.1;

  double x_impact_lx = 0.0;
  double x_impact_ly = 0.0;
  for (int i = 0; i < x_encs.size(); ++i) {
    auto curr = x_encs[i].first->get_deg();
    double d_raw =
        x_encs[i].second.travel_per_deg * (curr - prev_x_enc_vals[i]);
    prev_x_enc_vals[i] = curr;
    if (is_low_turn) {
      x_impact_lx += d_raw;
    } else {
      double tmp = (d_raw / in_rad(dh) - x_encs[i].second.offset);
      x_impact_lx += std::sin(in_rad(dh)) * tmp;
      x_impact_ly += (std::cos(in_rad(dh)) - 1) * tmp;
    }
  }
  if (x_encs.size() > 0) {
    x_impact_lx /= x_encs.size();
    x_impact_ly /= x_encs.size();
  }

  double y_impact_lx = 0.0;
  double y_impact_ly = 0.0;
  for (int i = 0; i < y_encs.size(); ++i) {
    auto curr = y_encs[i].first->get_deg();
    double d_raw =
        y_encs[i].second.travel_per_deg * (curr - prev_y_enc_vals[i]);
    prev_y_enc_vals[i] = curr;
    if (is_low_turn) {
      y_impact_ly += d_raw;
    } else {
      double tmp = (d_raw / in_rad(dh) + y_encs[i].second.offset);
      y_impact_lx += (1 - std::cos(in_rad(dh))) * tmp;
      y_impact_ly += std::sin(in_rad(dh)) * tmp;
    }
  }
  if (y_encs.size() > 0) {
    y_impact_lx /= y_encs.size();
    y_impact_ly /= y_encs.size();
  }

  auto dx_l = x_impact_lx + y_impact_lx;
  auto dy_l = x_impact_ly + y_impact_ly;
  auto dx_g = dx_l * std::cos(in_rad(prev_heading)) +
              dy_l * std::sin(in_rad(prev_heading));
  auto dy_g = -dx_l * std::sin(in_rad(prev_heading)) +
              dy_l * std::cos(in_rad(prev_heading));

  prev_heading = pose.heading();

  if (enabled) {
    lock();
    pose.x += dx_g;
    pose.y += dy_g;
    pose.h = mod(pose.h + dh, 360.0);
    unlock();
  }
}

ImuOdometry::Builder &
ImuOdometry::Builder::with_gyro(std::shared_ptr<AbstractGyro> igyro) {
  gyro = std::move(igyro);
  return *this;
}
ImuOdometry::Builder &
ImuOdometry::Builder::with_x_enc(std::shared_ptr<AbstractEncoder> encoder,
                                 encoder_conf_s conf) {
  x_encs.emplace_back(std::move(encoder), conf);
  return *this;
}
ImuOdometry::Builder &
ImuOdometry::Builder::with_y_enc(std::shared_ptr<AbstractEncoder> encoder,
                                 encoder_conf_s conf) {
  y_encs.emplace_back(std::move(encoder), conf);
  return *this;
}

std::shared_ptr<ImuOdometry> ImuOdometry::Builder::build() {
  if (gyro == nullptr) {
    return nullptr;
  }
  for (auto pair : x_encs) {
    if (pair.first == nullptr) {
      return nullptr;
    }
  }
  for (auto pair : y_encs) {
    if (pair.first == nullptr) {
      return nullptr;
    }
  }

  std::shared_ptr<ImuOdometry> odom(new ImuOdometry());
  odom->prev_timestamp = pros::millis();
  odom->gyro = gyro;
  odom->x_encs = x_encs;
  odom->y_encs = y_encs;
  odom->prev_x_enc_vals = std::vector<double>(x_encs.size(), 0.0);
  odom->prev_y_enc_vals = std::vector<double>(y_encs.size(), 0.0);
  odom->pose = pose_s{0, 0, 0};

  return odom;
}