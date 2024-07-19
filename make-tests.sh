#!/usr/bin/env bash
g++ test-src/test-utils.cpp src/subzerolib/api/util/controls.cpp src/subzerolib/api/util/math.cpp -isystemvendor/include -Iinclude -o test-output/test-utils.bin
g++ test-src/test-plot.cpp src/subzerolib/api/geometry/point.cpp src/subzerolib/api/geometry/segment.cpp src/subzerolib/api/geometry/circle.cpp src/subzerolib/api/util/helper.cpp src/subzerolib/api/spline/catmull-rom.cpp -isystemvendor/include -Iinclude -DEIEN_DONT_VECTORIZE -o test-output/test-plot.bin
g++ test-src/test-kf.cpp src/subzerolib/api/filter/kalman-filter.cpp -isystemvendor/include -Iinclude -o test-output/test-kf.bin
g++ test-src/test-mp.cpp src/subzerolib/api/trajectory/motion-profile/trapezoidal-motion-profile.cpp -Iinclude -o test-output/test-mp.bin
