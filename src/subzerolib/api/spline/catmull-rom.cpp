#include "subzerolib/api/spline/catmull-rom.hpp"
#include "subzerolib/api/spline/spline.hpp"
#include <algorithm>

CatmullRomSpline::CatmullRomSpline(std::vector<point_s> icontrol_points)
    : control_points(icontrol_points) {
  calculate_bernstein_coeffs();
}

CatmullRomSpline::CatmullRomSpline(
    std::initializer_list<point_s> icontrol_points)
    : CatmullRomSpline(std::vector<point_s>{icontrol_points}) {}

CatmullRomSpline::CatmullRomSpline(std::vector<pose_s> icontrol_points) {
  control_points.resize(icontrol_points.size());
  transform(icontrol_points.begin(),
            icontrol_points.end(),
            control_points.begin(),
            [](pose_s pose) -> point_s { return pose; });
  calculate_bernstein_coeffs();
}

CatmullRomSpline::CatmullRomSpline(
    std::initializer_list<pose_s> icontrol_points)
    : CatmullRomSpline(std::vector<pose_s>{icontrol_points}) {}

void CatmullRomSpline::calculate_bernstein_coeffs() {
  bernstein_coeffs.clear();
  if (control_points.size() >= 4) {
    for (int end_i = 3; end_i < control_points.size(); ++end_i) {
      Eigen::Matrix<double, 4, 2> ctrl_matrix;
      for (int inc = 3; inc >= 0; --inc) {
        ctrl_matrix.row(3 - inc)[0] = control_points[end_i - inc].x;
        ctrl_matrix.row(3 - inc)[1] = control_points[end_i - inc].y;
      }
      bernstein_coeffs.push_back(characteristic_matrix * ctrl_matrix);
    };
  }
}

std::vector<point_s> CatmullRomSpline::sample_coordinates(int count) {
  std::vector<point_s> sampled_points(count);
  if (count < 1) {
    return sampled_points;
  }
  double step = (control_points.size() - 3) / (count - 1.0);
  double u = 0.0;
  for (int i = 0; i < count; ++i) {
    sampled_points[i] = get_pos(u);
    u += step;
  }

  return sampled_points;
}

std::vector<spline_point_s> CatmullRomSpline::sample_kinematics(int count) {
  std::vector<spline_point_s> sampled_points(count);
  if (count < 1) {
    return sampled_points;
  }
  double step = (control_points.size() - 3) / (count - 1.0);
  double u = 0.0;
  for (int i = 0; i < count; ++i) {
    auto pos = get_pos(u);
    double s = 0.0;
    if (i > 0) {
      s = sampled_points[i - 1].s + pos.dist(sampled_points[i - 1]);
    }
    auto vel = get_vel(u);
    auto accel = get_accel(u);

    sampled_points[i] =
        spline_point_s{pos.x, pos.y, s, vel.x, vel.y, accel.x, accel.y};
    u += step;
  }

  return sampled_points;
}

point_s CatmullRomSpline::get_val(Eigen::Matrix<double, 1, 4> t_row, double u) {
  point_s out{0, 0};
  if (u < 0.0 || bernstein_coeffs.size() == 0) {
    return out;
  }
  int whole = (int)std::floor(u) - (u >= bernstein_coeffs.size());
  Eigen::Matrix<double, 1, 2> product = t_row * bernstein_coeffs[whole];

  out.x = product(0);
  out.y = product(1);
  return out;
}

point_s CatmullRomSpline::get_pos(double u) {
  double t = u - std::floor(u);
  t += (u >= bernstein_coeffs.size());
  return get_val(Eigen::Matrix<double, 1, 4>{1, t, t * t, t * t * t}, u);
}

point_s CatmullRomSpline::get_vel(double u) {
  double t = u - std::floor(u);
  t += (u >= bernstein_coeffs.size());
  return get_val(Eigen::Matrix<double, 1, 4>{0, 1, 2 * t, 3 * t * t}, u);
}

point_s CatmullRomSpline::get_accel(double u) {
  double t = u - std::floor(u);
  t += (u >= bernstein_coeffs.size());
  return get_val(Eigen::Matrix<double, 1, 4>{0, 0, 2, 6 * t}, u);
}

void CatmullRomSpline::pad_velocity(point_s v0, point_s vf) {
  if (control_points.size() >= 2) {
    point_s start_padding = *(control_points.begin() + 1) - 2 * v0;
    point_s end_padding = *(control_points.end() - 2) + 2 * vf;
    control_points.insert(control_points.begin(), start_padding);
    control_points.push_back(end_padding);
  }
  calculate_bernstein_coeffs();
}
