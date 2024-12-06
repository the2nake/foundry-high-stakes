#include "subzerolib/api/trajectory/spline-trajectory.hpp"
#include "subzerolib/api/geometry/trajectory-point.hpp"
#include "subzerolib/api/spline/spline.hpp"
#include "subzerolib/api/trajectory/motion-profile/linear-motion-profile.hpp"
#include "subzerolib/api/util/math.hpp"
#include "subzerolib/api/util/search.hpp"

#include <limits>

double SplineTrajectory::get_duration() { return vec.back().t; }
double SplineTrajectory::get_length() { return vec.back().s; }

trajectory_point_s SplineTrajectory::get_at_time(double t) {
  int next_i = binary_search<trajectory_point_s, double>(
      vec, t, [](trajectory_point_s p) -> double { return p.t; });
  if (next_i <= 0)
    return vec.front();
  if (next_i >= vec.size())
    return vec.back();
  double f = (t - vec[next_i - 1].t) / (vec[next_i].t - vec[next_i - 1].t);
  if (std::isnan(f)) {
    return vec[next_i];
  }
  return lerp(vec[next_i - 1], vec[next_i], f);
}

trajectory_point_s SplineTrajectory::get_at_distance(double s) {
  int next_i = binary_search<trajectory_point_s, double>(
      vec, s, [](trajectory_point_s p) -> double { return p.s; });
  if (next_i <= 0)
    return vec.front();
  if (next_i >= vec.size())
    return vec.back();
  double f = (s - vec[next_i - 1].s) / (vec[next_i].s - vec[next_i - 1].s);
  if (std::isnan(f)) {
    return vec[next_i];
  }
  return lerp(vec[next_i - 1], vec[next_i], f);
}

SplineTrajectory::SplineTrajectory(
    std::shared_ptr<Spline> spline,
    std::shared_ptr<LinearMotionProfile> i_profile,
    std::shared_ptr<Model> i_model,
    int sample_count) {
  this->model = i_model;
  this->profile = i_profile;

  this->spline_points = spline->sample_kinematics(sample_count);
  this->profile->set_resolution(0.005);
  this->profile->generate(this->spline_points.back().s);

  // double pass: caclulate model velocity for every point, minimise between
  // that and profile velocities, same with accel, decel

  // then, going forward, restrain increasing velocities, if velocities increase
  // too fast, slow down then, going backward, restrain decreasing velocities,
  // if velocity is two high, slow it down
  forward_pass();
  reverse_pass();

  // limit all velocities in the spline or copy over
  apply_constraints();

  // do check at the end if the profile is achievable
  // check performance on vex brain
}

// set max velocities
void SplineTrajectory::forward_pass() {
  this->max_vels.clear();
  this->max_vels.reserve(spline_points.size());

  for (auto curr = spline_points.begin(); curr != spline_points.end(); ++curr) {
    auto profile_constraints = this->profile->get_point_at_distance(curr->s);
    auto model_constraints = this->model->get_constraints(curr->curvature());

    double max_vel = std::min(profile_constraints.v, model_constraints.max_vel);
    if (curr != this->spline_points.begin() && !this->max_vels.empty()) {
      // minimise max vel based on if it is reachable

      // vf^2 = v0^2 + 2adx ==> vf = sqrt(v0 * v0 + 2 * a * dx)
      auto prev = curr - 1;
      double dx = curr->s - prev->s;

      double v0 = this->max_vels.back();
      double a = model_constraints.max_accel;

      double vf = std::sqrt(v0 * v0 + 2 * a * dx);
      max_vel = std::min(max_vel, vf);
    }

    // ? use abs_min

    this->max_vels.emplace_back(max_vel);
  }
}

void SplineTrajectory::reverse_pass() {
  // skip the beginning point because it cannot be constrained
  for (auto succs =
           std::pair{this->spline_points.rbegin(), this->max_vels.rbegin()};
       succs.first != this->spline_points.rend() - 2 &&
       succs.second != this->max_vels.rend() - 2;
       ++succs.first, ++succs.second) {
    auto curr_sp = succs.first + 1;
    // v0 = sqrt(vf * vf - 2 * a * dx)
    double dx = succs.first->s - curr_sp->s;
    double vf = *succs.second;

    double a = -this->model->get_constraints(curr_sp->curvature()).max_decel;
    double v0 = std::sqrt(vf * vf - 2 * a * dx);

    auto &curr_mv = *(succs.second + 1);
    curr_mv = std::min(v0, curr_mv);
  }
}

int SplineTrajectory::Builder::find_pose_index(pose_s pose) {
  double min_d = std::numeric_limits<double>::max();
  int return_index = 0;
  for (int i = 0; i < traj.size(); ++i) {
    double dist = pose.dist(traj[i]);
    if (dist < min_d) {
      return_index = i;
      min_d = dist;
    }
  }
  return return_index;
}

void SplineTrajectory::apply_constraints() {
  this->vec.reserve(spline_points.size());
  std::for_each(this->spline_points.begin(),
                this->spline_points.end(),
                [this](spline_point_s p) {
                  trajectory_point_s traj_point{p};
                  // calculate h before generating vh
                  traj_point.h = 90 - in_deg(atan2(p.vy, p.vx));
                  if (!vec.empty()) {
                    traj_point.h =
                        vec.back().h + shorter_turn(vec.back().h, traj_point.h);
                  } else {
                    traj_point.h = mod(traj_point.h, 360.0);
                  }
                  this->vec.emplace_back(traj_point);
                });

  for (auto it = std::pair(this->vec.begin(), this->max_vels.begin());
       it.first != this->vec.end() && it.second != this->max_vels.end();
       ++it.first, ++it.second) {
    auto &curr = *it.first;
    auto &prev = *(it.first - 1);

    // apply velocity
    double mult = *it.second / curr.v();
    curr.vx *= mult;
    curr.vy *= mult;

    // calculate time using kinematics
    if (it.first == this->vec.begin()) {
      curr.t = 0.0;
    } else {
      double sum_v = curr.v() + prev.v();
      if (rougheq(sum_v, 0.0)) {
        curr.t = K_EPSILON;
        // issue warning: probably spline generated incorrectly
      } else {
        // t = 2 * dx / (v0 + vf)
        double dx = curr.s - prev.s;
        curr.t = 2 * dx / sum_v;
      }

      // calculate vh as a tangent
      prev.vh = shorter_turn(prev.h, curr.h) / curr.t;
    }

    curr.t += prev.t;
  }
}

SplineTrajectory::Builder &
SplineTrajectory::Builder::with_spline(std::shared_ptr<Spline> i_spline,
                                       std::vector<pose_s> i_control_points) {
  if (i_spline != nullptr) {
    spline = i_spline;
    control_points = std::move(i_control_points);
  }

  return *this;
}

SplineTrajectory::Builder &SplineTrajectory::Builder::with_motion_profile(
    std::shared_ptr<LinearMotionProfile> i_profile) {
  if (i_profile != nullptr) {
    profile = i_profile;
  }

  return *this;
}

SplineTrajectory::Builder &
SplineTrajectory::Builder::with_chassis(std::shared_ptr<Chassis> i_chassis) {
  if (i_chassis != nullptr) {
    chassis = i_chassis;
  }

  return *this;
}

void SplineTrajectory::Builder::sample_spline() {
  spline_points = spline->sample_kinematics(sample_count);
  traj.resize(spline_points.size());
  for (int i = 0; i < spline_points.size(); ++i) {
    traj[i] = spline_points[i];
  }
}

void SplineTrajectory::Builder::get_control_indices() {
  for (auto &ctrl_point : control_points) {
    int i = find_pose_index(ctrl_point);
    auto &traj_ctrl = traj[i];
    traj_ctrl.x = ctrl_point.x;
    traj_ctrl.y = ctrl_point.y;
    traj_ctrl.h = ctrl_point.h;
    ctrl_is.push_back(i);
  }
}

void SplineTrajectory::Builder::apply_motion_profile() {
  traj.back().vx = 0;
  traj.back().vy = 0;
  profile->generate(traj.back().s);
  for (int i = 0; i < traj.size(); ++i) {
    auto lin_point = profile->get_point_at_distance(traj[i].s);
    traj[i].t = lin_point.t;

    if (i < traj.size() - 1) {
      auto &curr = traj[i];
      auto &next = traj[i + 1];

      double dx = next.x - curr.x;
      double dy = next.y - curr.y;
      double ds = hypot(dx, dy); // don't use s here

      curr.vx = lin_point.v / ds * dx;
      curr.vy = lin_point.v / ds * dy;
    }
  }
}

double get_a(double x, double v0, double vf) {
  return (vf * vf - v0 * v0) / (2 * x);
}

void SplineTrajectory::Builder::constrain_2d_accel() {
  // calculate velocity ranges for each point
  //   use acceleration kinematic formula with distance parameterization
  const double m_vel = profile->get_max_vel();
  const double m_accel = profile->get_max_accel();

  std::vector<double> max_vs(traj.size());
  max_vs[0] = 0;
  max_vs.back() = 0;

  // forward pass
  for (int i = 1; i < max_vs.size() - 1; ++i) {
    if (max_vs[i] > m_vel) {
      max_vs[i] = m_vel;
    }

    // m_accel ^ 2 = accel_x ^ 2 + accel_y ^ 2
    // m_accel ^ 2 = (vfx*vfx - v0x*v0x)^2 / 4sx^2 + (vfy*vfy - v0y*v0y)^2 /
    // 4sy^2 4 * m_accel * m_accel * sx * sx * sy * sy = sy^2(vfx*vfx -
    // v0x*v0x)^2 + sx^2(vfy*vfy - v0y*v0y)^2
    double accel = std::hypot(
        get_a(traj[i].x - traj[i - 1].x, traj[i - 1].vx, traj[i].vx),
        get_a(traj[i].y - traj[i - 1].y, traj[i - 1].vy, traj[i].vy));

    // BUG: not maxing out on acceleration in prior steps causes infinite hang
    while (accel > m_accel) {
      double v_scale = 0.95;
      traj[i].vx *= v_scale;
      traj[i].vy *= v_scale;
      accel = std::hypot(
          get_a(traj[i].x - traj[i - 1].x, traj[i - 1].vx, traj[i].vx),
          get_a(traj[i].y - traj[i - 1].y, traj[i - 1].vy, traj[i].vy));
      printf("[%d] a=%f\n", i, accel);
    }

    // double v_scale = max_vs[i] / traj[i].v();
    // traj[i].vx *= v_scale;
    // traj[i].vy *= v_scale;
  }

  for (int i = 1; i < traj.size(); ++i) {
    traj[i].t = traj[i - 1].t + (traj[i].s - traj[i - 1].s) / traj[i].v();
  }

  for (int i = 0; i < max_vs.size(); ++i) {
    printf("%f %f v=%f\n", traj[i].x, traj[i].y, max_vs[i]);
  }
}

bool SplineTrajectory::Builder::is_accel_broken(int i) {
  double m_accel = profile->get_max_accel();
  double m_decel = profile->get_max_decel();
  if (i >= 0 && i < traj.size()) {
    double accel = get_accel(i);
    return accel > m_accel || accel < -m_decel;
  }
  bool output = false;
  for (int i = 1; i < traj.size(); ++i) {
    if (is_accel_broken(i)) {
      printf("[%d] broken: a=%6.2f\n", i, get_accel(i));
      output = true;
    }
  }
  return output;
}

double SplineTrajectory::Builder::get_accel(int i) {
  clamp_val<int>(i, 0, traj.size() - 2);
  auto point = traj[i + 1];
  auto prev = traj[i];
  double accel_x = (point.vx - prev.vx) / (point.t - prev.t);
  double accel_y = (point.vy - prev.vy) / (point.t - prev.t);
  return std::hypot(accel_x, accel_y);
}

void SplineTrajectory::Builder::generate_heading() {
  if (b_mode == heading_mode_e::path) {
    for (int i = 0; i < traj.size() - 1; ++i) {
      auto &curr = traj[i];
      auto &next = traj[i + 1];
      curr.h = 90.0 - in_deg(std::atan2(next.y - curr.y, next.x - curr.x));
    }
    return;
  }

  int j = 0, i0 = 0, i1 = 0;
  double dh = 0, dt = 0, max_a = 0.0;
  for (int i = 0; i < traj.size(); ++i) {
    bool is_ctrl =
        std::find(ctrl_is.begin(), ctrl_is.end(), i) != ctrl_is.end();
    if (is_ctrl) {
      if (j++ < ctrl_is.size()) {
        i0 = ctrl_is[j - 1];
        i1 = ctrl_is[j];
        continue;
      }
      break;
    }
    dh = shorter_turn(traj[i0].h, traj[i1].h, 360.0);
    dt = traj[i1].t - traj[i0].t;
    max_a = 4 * dh / (dt * dt);

    double t_i = traj[i].t - traj[i0].t;
    traj[i].h = traj[i0].h;
    if (t_i < dt / 2.0) {
      traj[i].h += max_a * t_i * t_i * 0.5;
    } else {
      t_i -= dt / 2.0;
      traj[i].h += 0.5 * dh + (2 * dh * t_i / dt) - (0.5 * max_a * t_i * t_i);
    }

    if (i >= 1) {
      traj[i - 1].vh = shorter_turn(traj[i - 1].h, traj[i].h, 360.0) /
                       (traj[i].t - traj[i - 1].t);
    }
  }
}

void SplineTrajectory::Builder::apply_model_constraints() {
  std::vector<double> v_max = chassis->get_wheel_max();
  // used to reformulate time points
  std::vector<double> dts(traj.size());
  dts.front() = 0.0;
  for (int i = 1; i < traj.size(); ++i) {
    auto &p = traj[i];

    point_s local = rotate_acw(p.vx, p.vy, p.h);
    auto v_wheel = chassis->get_wheel_vels(local.x, local.y, in_rad(p.vh));

    // used to accumulate scaling ratios
    double vel_scale = 1.0;
    for (int j = 0; j < v_wheel.size(); ++j) {
      double mag = std::abs(v_wheel[j]);
      double max = std::abs(v_max[j]);

      // scale uniformly if exceeding
      if (mag > max) {
        double scale = max / mag;
        for (auto &v : v_wheel) {
          v *= scale;
        }
        vel_scale *= 0.999999 * scale;
      }
    }
    double dt = traj[i].t - traj[i - 1].t;
    dts[i] = dt / vel_scale;
    p.vx *= vel_scale;
    p.vy *= vel_scale;
    p.vh *= vel_scale;
  }

  // integrate new time deltas
  for (int i = 1; i < traj.size(); ++i) {
    traj[i].t = traj[i - 1].t + dts[i];
  }
}

std::shared_ptr<SplineTrajectory> SplineTrajectory::Builder::build() {
  sample_spline();
  get_control_indices();

  apply_motion_profile();
  // constrain_2d_accel();

  generate_heading();

  apply_model_constraints();

  is_accel_broken();

  std::shared_ptr<SplineTrajectory> ptr{new SplineTrajectory()};
  ptr->vec = traj;

  return ptr;
}
