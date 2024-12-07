#pragma once

#include "subzerolib/api/chassis/chassis.hpp"
#include "subzerolib/api/chassis/model/model.hpp"
#include "subzerolib/api/geometry/pose.hpp"
#include "subzerolib/api/geometry/trajectory-point.hpp"
#include "subzerolib/api/spline/spline.hpp"
#include "subzerolib/api/trajectory/motion-profile/linear-motion-profile.hpp"
#include "subzerolib/api/trajectory/trajectory.hpp"

#include <memory>
#include <vector>

class SplineTrajectory : public Trajectory {
public:
  enum class heading_mode_e { path, pose };

  // use templates instead of polymorphism?
  SplineTrajectory(std::shared_ptr<Spline> spline,
                   std::shared_ptr<LinearMotionProfile> profile,
                   std::shared_ptr<Model> model,
                   int sample_count = 200);

  double get_duration() override;
  double get_length() override;

  trajectory_point_s get_at_time(double t) override;
  trajectory_point_s get_at_distance(double s) override;
  std::vector<trajectory_point_s> get_profile() override;

  void print() {
    for (auto &p : vec) {
      printf("t=%6.3f s=%6.3f (%5.2f,%5.2f) h=%6.1f vh=%4.0f vx=%5.2f vy=%5.2f "
             "v=%5.2f\n",
             p.t,
             p.s,
             p.x,
             p.y,
             p.h,
             p.vh,
             p.vx,
             p.vy,
             std::hypot(p.vx, p.vy));
    }
  }

private:
  SplineTrajectory() {}

  void forward_pass();
  void reverse_pass();
  void apply_constraints();

  std::vector<spline_point_s> spline_points;
  std::vector<trajectory_point_s> vec;
  std::vector<double> max_vels;

  std::shared_ptr<LinearMotionProfile> profile;
  std::shared_ptr<Model> model;

public:
  class Builder {
  public:
    Builder(heading_mode_e i_mode = SplineTrajectory::heading_mode_e::path,
            int i_sample_count = 200)
        : b_mode(i_mode), sample_count(i_sample_count) {}

    Builder &with_spline(std::shared_ptr<Spline> i_spline,
                         std::vector<pose_s> i_control_points);

    Builder &with_motion_profile(std::shared_ptr<LinearMotionProfile> profile);

    Builder &with_chassis(std::shared_ptr<Chassis> i_chassis);

    std::shared_ptr<SplineTrajectory> build();

    const heading_mode_e b_mode;
    const int sample_count;

  private:
    int find_pose_index(pose_s pose);

    std::shared_ptr<Spline> spline = nullptr;
    std::shared_ptr<LinearMotionProfile> profile = nullptr;
    std::shared_ptr<Chassis> chassis = nullptr;

    std::vector<pose_s> control_points;
    std::vector<trajectory_point_s> traj;

    void sample_spline();
    std::vector<spline_point_s> spline_points;

    void get_control_indices();
    std::vector<int> ctrl_is;

    void apply_motion_profile();
    void constrain_2d_accel();
    void generate_heading();
    void apply_model_constraints();

    bool is_accel_broken(int i = -1);
    double get_accel(int i = 1);
  };
};
