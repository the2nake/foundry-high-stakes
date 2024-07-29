#include "subzerolib/api/spline/catmull-rom.hpp"
#include "subzerolib/api/spline/spline.hpp"
#include "subzerolib/api/trajectory/motion-profile/linear-motion-profile.hpp"
#include "subzerolib/api/trajectory/motion-profile/trapezoidal-motion-profile.hpp"
#include "subzerolib/api/util/math.hpp"

#include <cmath>
#include <limits>

struct trajectory_point_s {
  trajectory_point_s(double i_t = 0,
                     double i_s = 0,
                     double i_x = 0,
                     double i_vx = 0,
                     double i_y = 0,
                     double i_vy = 0,
                     double i_h = 0,
                     double i_vh = 0)
      : t(i_t), s(i_s), x(i_x), vx(i_vx), y(i_y), vy(i_vy), h(i_h), vh(i_vh) {}
  trajectory_point_s(spline_point_s &point)
      : s(point.s), x(point.x), vx(point.vx), y(point.y), vy(point.vy) {}

  double t = 0;
  double s = 0;
  double x = 0;
  double vx = 0;
  double y = 0;
  double vy = 0;
  double h = 0;
  double vh = 0;
};

int find_pose_index(std::vector<spline_point_s> &vec, pose_s pose) {
  double min_d = std::numeric_limits<double>::max();
  double index = 0.0;
  for (int i = 0; i < vec.size(); ++i) {
    double dist = pose.dist(vec[i].point());
    if (dist < min_d) {
      index = i;
      min_d = dist;
    }
  }
  double dx = pose.x - vec[index].x;
  double dy = pose.y - vec[index].y;
  bool same_dir = vec[index].vx * dx + dy * vec[index].vy > 0;
  return index + int(same_dir);
}

int main() {
  // set up a linear motion profile
  LinearMotionProfile *generator = new TrapezoidalMotionProfile{1.85, 6, 6};
  generator->set_resolution(0.01);

  // generate the curve using a catmull rom spline
  std::vector<pose_s> ctrl = {
      pose_s{  0.0,  0.0,   0.0},
      pose_s{  0.4,  0.6,  45.0},
      pose_s{ -0.2,  0.6,  60.0},
      pose_s{-0.75, 0.75, -45.0}
  };
  CatmullRomSpline spline(ctrl);
  spline.pad_velocity({0.5, 0.5}, {-0.25, 0.25});
  std::vector<spline_point_s> sampled = spline.sample_kinematics(300);
  std::vector<trajectory_point_s> profile(sampled.size());
  for (int i = 0; i < sampled.size(); ++i) {
    profile[i] = sampled[i];
  }
  std::vector<int> ctrl_indices;
  for (auto &c : ctrl) {
    int i = find_pose_index(sampled, c);
    profile[i].x = c.x;
    profile[i].y = c.y;
    profile[i].h = c.h;
    ctrl_indices.push_back(i);
  }

  // apply profile to path
  generator->generate(profile.back().s);
  // dynamic_cast<TrapezoidalMotionProfile *>(generator)->print();
  for (int i = 0; i < profile.size(); ++i) {
    LinearMotionProfile::profile_point_s motion =
        generator->get_point_at_distance(profile[i].s);
    profile[i].t = motion.t;
    if (i < profile.size() - 1) {
      double dx = profile[i + 1].x - profile[i].x;
      double dy = profile[i + 1].y - profile[i].y;
      double ds = hypot(dx, dy); // don't use s here
      profile[i].vx = motion.v / ds * dx;
      profile[i].vy = motion.v / ds * dy;
    }
  }
  // the endpoint is assumed to be a control point
  profile.back().vx = 0;
  profile.back().vy = 0;

  // lerp headings and angular velocity
  for (int i = 0; i < profile.size(); ++i) {
    if (std::find(ctrl_indices.begin(), ctrl_indices.end(), i) ==
        ctrl_indices.end()) {
      int start_index = 0;
      int end_index = 0;
      for (int j = 1; j < ctrl_indices.size(); ++j) {
        if (ctrl_indices[j] > i) {
          start_index = ctrl_indices[j - 1];
          end_index = ctrl_indices[j];
          break;
        }
      }
      double long_s = profile[end_index].s - profile[start_index].s;
      double curr_s = profile[i].s - profile[start_index].s;
      double factor = curr_s / long_s;
      double begin_h = profile[start_index].h;
      double end_h = profile[end_index].h;
      profile[i].h = begin_h + factor * shorter_turn(begin_h, end_h, 360.0);
      /*
      auto &p = profile[i];
      printf("i=%d s=%.3f (%.2f, %.2f) h=%.0f factor=%.3f i0=%d i1=%d\n",
             i,
             p.s,
             p.x,
             p.y,
             p.h,
             factor,
             start_index,
             end_index);
             */
    } else {
      /*
      auto &p = profile[i];
      printf("i=%d s=%.3f (%.2f, %.2f) h=%.0f\n", i, p.s, p.x, p.y, p.h);
      */
    }
  }
  profile.back().vh = 0;

  // generate vh
  for (int i = 0; i < profile.size() - 1; ++i) {
    double dt = profile[i + 1].t - profile[i].t;
    // printf("%f\n", dt);
    profile[i].vh = shorter_turn(profile[i].h, profile[i + 1].h, 360.0) / dt;
  }

  for (trajectory_point_s &p : profile) {
    printf("t=%.3f s=%.3f (%.2f, %.2f) h=%.3f vh=%.1f vx=%.2f vy=%.2f v=%.2f\n",
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

  // TODO: clamp with model constraints
  // TODO: clamp with kinematic constraints
  // TODO: unit test with integration
  return 0;
}
