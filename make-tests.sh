#!/usr/bin/env bash

# TODO: create CMake configuration

GEOMETRY_CPP=src/subzerolib/api/geometry/*.cpp

g++ -std=c++20 test-src/test-utils.cpp src/subzerolib/api/util/controls.cpp src/subzerolib/api/util/math.cpp -isystemvendor/include -Iinclude -DEIGEN_DONT_VECTORIZE -o test-output/test-utils.bin
g++ -std=c++20 test-src/test-pp.cpp src/subzerolib/api/geometry/point.cpp src/subzerolib/api/geometry/segment.cpp src/subzerolib/api/geometry/circle.cpp src/subzerolib/api/util/helper.cpp src/subzerolib/api/spline/catmull-rom.cpp -isystemvendor/include -Iinclude -DEIEN_DONT_VECTORIZE -o test-output/test-pp.bin
g++ -std=c++20 test-src/test-kf.cpp src/subzerolib/api/filter/kalman-filter.cpp -isystemvendor/include -Iinclude -DEIGEN_DONT_VECTORIZE -o test-output/test-kf.bin
g++ -std=c++20 test-src/test-mp.cpp src/subzerolib/api/chassis/model/tank-model.cpp src/subzerolib/api/spline/spline.cpp src/subzerolib/api/trajectory/spline-trajectory.cpp src/subzerolib/api/trajectory/motion-profile/trapezoidal-motion-profile.cpp src/subzerolib/api/util/math.cpp $GEOMETRY_CPP src/subzerolib/api/util/helper.cpp src/subzerolib/api/spline/catmull-rom.cpp -isystemvendor/include -Iinclude -DEIGEN_DONT_VECTORIZE -o test-output/test-mp.bin
