# 3818D High Stakes Codebase

Watch our reveal video!

<iframe width="560" height="315" src="https://www.youtube.com/embed/y0sAqeBquyk?si=TzQgcpja-kDgG06w" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Features

### Teleop

- Control code for tank, asterisk and X drivetrain types

### Motion Algorithms

- Easy-to-use PID classes
- Generic odometry-based PID motion for holonomic drivetrains
- Non-overlapping Pure Pursuit implementation
  - with tracked position interpolation to prevent point skipping
- 2D Motion Profiling for holonomic drives
  - with support for any linear profile
  - with angular acceleration minimization

- Path generation using uniform Catmull-Rom spline
  - centripetal + chordal coming soon
- Heading interpolation for splines
  - lerp from control points
- Trapezoidal linear motion profile generation
  - time and distance parameterization

### Sensors

- Abstractions for gyroscopes and rotation encoders
- Filtered gyroscope class using modular arithmetic averages
- Custom 2-tracker odometry algorithm for higher tracking accuracy
- Sensor fusion with 2 IMUs with the KFOdometry class

### Math and Logic

- Extensible state machine implementation
- Reliable ExitCondition class
- Linear and angular balancing for teleop "arcade" style movement

### Utils

- Logging with colour support
- Generic `AutoUpdater` for updating odometry, PID, exit conditions, etc.
- A variety of miscellaneous functions such as `shorter_turn` and `clamp_distance`

## Dependencies

### Tests

`ccache g++`

### plot.py

`ffmpeg`

### Python (via pip)

`pros-cli matplotlib PyQt6`
