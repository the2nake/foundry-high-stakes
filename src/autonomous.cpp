#include "devices.hpp"
#include "main.h"
#include "subzerolib/api/spline/catmull-rom.hpp"
#include "subzerolib/api/trajectory/motion-profile/linear-motion-profile.hpp"
#include "subzerolib/api/trajectory/motion-profile/trapezoidal-motion-profile.hpp"
#include "subzerolib/api/trajectory/spline-trajectory.hpp"

/*
void autonomous() {
  // controller->move_to_pose({0.3, 0.3, 270});
  // controller->move_to_pose({-0.6, 0.5, 315});

  std::shared_ptr<ExitCondition<double>> pass_cond{
      new ExitCondition<double>{{0, 0.06}, 100}
  };

  std::shared_ptr<ExitCondition<double>> end_cond{
      new ExitCondition<double>{{0, 0.02}, 200}
  };

  std::vector<pose_s> ctrl = {
      pose_s{  0.0,  0.0,   0.0},
      pose_s{  0.4,  0.6,  45.0},
      pose_s{ -0.2,  0.6,  60.0},
      pose_s{-0.75, 0.75, -45.0}
  };
  std::shared_ptr<CatmullRomSpline> spline{new CatmullRomSpline{ctrl}};
  spline->pad_velocity({0.5, 0.5}, {-0.25, 0.25});

  std::shared_ptr<TrapezoidalMotionProfile> profile{
      new TrapezoidalMotionProfile{chassis->get_max_vel(), 5}
  };

  auto traj =
      SplineTrajectory::Builder(SplineTrajectory::heading_mode_e::pose, 400)
          .with_spline(spline, ctrl)
          .with_chassis(chassis)
          .with_motion_profile(profile)
          .build();
}*/
