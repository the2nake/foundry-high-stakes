#include "subzerolib/api/filter/kalman-filter.hpp"
#include "subzerolib/api/odometry/imu-odometry.hpp"

/// Requirements for the filter:
///
/// state vector of form [x, vx, y, vy, h, vh]
/// measurement vector of form [vx, vy, h, vh]
/// control vector form [ax, ay]
class KFOdometry : public ImuOdometry, private KalmanFilter {
public:
  virtual ~KFOdometry() {}

  /// @brief sets the heading of the odometry module
  /// @param heading the desired heading
  void set_heading(double heading) override;

  /// @brief sets the position of the odometry module
  /// @param x the desired position's x coordinate
  /// @param y the desired position's y coordinate
  void set_position(double x, double y) override;

  /// @brief get the current pose
  /// @returns the pose measurement
  pose_s get_pose() override;

  /// @brief get the current velocity
  /// @returns the velocity measurement
  pose_s get_vel() override;

  /// @brief trigger an update tick
  void update() override;

  /// @brief enable/disable the module
  /// @param bool desired state
  void set_enabled(bool) override;

  /// @brief check if the module is enabled
  /// @returns whether odometry is active
  bool is_enabled() override;

  pose_s get_raw_pose() { return ImuOdometry::get_pose(); }
  pose_s get_raw_vel() { return ImuOdometry::get_vel(); }
  Eigen::VectorXd get_state() override { return KalmanFilter::get_state(); }
  Eigen::MatrixXd get_covariance() override {
    return KalmanFilter::get_covariance();
  }

private:
  KFOdometry(std::shared_ptr<ImuOdometry> i_odom,
             std::shared_ptr<KalmanFilter> i_kf)
      : ImuOdometry(std::move(*i_odom)), KalmanFilter(std::move(*i_kf)) {}

public:
  class Builder : public ImuOdometry::Builder, public KalmanFilter::Builder {
  public:
    Builder(uint nx, uint nu, uint nz) : KalmanFilter::Builder(nx, nu, nz) {}
    std::shared_ptr<KFOdometry> build();
  };
};
