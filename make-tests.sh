#!/usr/bin/env bash

# TODO: create CMake configuration

GEOMETRY_CPP="src/subzerolib/api/geometry/*.cpp src/subzerolib/api/util/math.cpp"

ccache g++ -std=c++20 test-src/test-utils.cpp src/subzerolib/api/util/controls.cpp $GEOMETRY_CPP -isystemvendor/include -Iinclude -DEIGEN_DONT_VECTORIZE -o test-output/test-utils.bin
ccache g++ -std=c++20 test-src/test-pp.cpp $GEOMETRY_CPP src/subzerolib/api/util/helper.cpp src/subzerolib/api/spline/catmull-rom.cpp -isystemvendor/include -Iinclude -DEIEN_DONT_VECTORIZE -o test-output/test-pp.bin
ccache g++ -std=c++20 test-src/test-kf.cpp src/subzerolib/api/filter/kalman-filter.cpp $GEOMETRY_CPP -isystemvendor/include -Iinclude -DEIGEN_DONT_VECTORIZE -o test-output/test-kf.bin
ccache g++ -std=c++20 test-src/test-mp.cpp src/subzerolib/api/chassis/model/tank-model.cpp src/subzerolib/api/spline/spline.cpp src/subzerolib/api/trajectory/spline-trajectory.cpp src/subzerolib/api/trajectory/motion-profile/trapezoidal-motion-profile.cpp $GEOMETRY_CPP src/subzerolib/api/util/helper.cpp src/subzerolib/api/spline/catmull-rom.cpp -isystemvendor/include -Iinclude -DEIGEN_DONT_VECTORIZE -o test-output/test-mp.bin
