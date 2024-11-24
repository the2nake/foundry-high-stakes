#include "subzerolib/api/util/controls.hpp"
#include "subzerolib/api/util/math.hpp"
#include "subzerolib/api/util/search.hpp"

#include <algorithm>
#include <stdio.h>
#include <vector>

void test(bool pass, const char *name) {
  if (pass) {
    printf("\033[0m[\033[1;32mpass\033[0m]: %s\n", name);
  } else {
    printf("\033[0m[\033[1;31mfail\033[0m]: %s\n", name);
  }
}

void test_vel_balance_with_tank() {
  // tank test
  std::vector<control_components_s> vels;
  vels.push_back(control_components_s{1.0, 1.0});
  vels.push_back(control_components_s{1.0, -1.0});
  double pref = 0.3;
  balance_vels(vels, 1.0, pref);
  test(std::abs(std::max_element(
                    vels.begin(),
                    vels.end(),
                    [](control_components_s a, control_components_s b) -> bool {
                      return std::abs(a.sum()) < std::abs(b.sum());
                    })
                    ->sum()) == 1.0,
       "tank velocity balancing - max velocity");
  bool passed = true;
  for (auto &vel : vels) {
    if (std::abs(vel.lin) < std::abs(vel.rot)) {
      passed = false;
    }

    if (not rougheq(std::abs(vel.rot) / (std::abs(vel.lin) + std::abs(vel.rot)),
                    pref)) {
      passed = false;
      printf("lin: %f, rot: %f\n", vel.lin, vel.rot);
    }
  }
  test(passed, "tank velocity balancing - preference to linear ratio");
}

void test_vel_sum_with_tank() {
  std::vector<control_components_s> vels;
  vels.push_back(control_components_s{1.0, 0.0});
  vels.push_back(control_components_s{1.0, 0.0});
  bool passed = true;
  balance_vels(vels, 1.0, 0.3);
  if (not rougheq(vels[0].sum(), 1.0)) {
    passed = false;
  }
  test(passed, "tank velocity balancing - correct summed velocity");
}

enum class tank_motors_e { left, right };
void test_map_vel_balance_with_tank() {
  // tank test
  std::map<tank_motors_e, control_components_s> vels;
  vels.emplace(tank_motors_e::left, control_components_s{1.0, 1.0});
  vels.emplace(tank_motors_e::right, control_components_s{1.0, -1.0});
  double pref = 0.4;
  balance_mapped_vels<tank_motors_e>(vels, 1.0, pref);
  test(std::abs(
           std::max_element(
               vels.begin(),
               vels.end(),
               [](std::pair<tank_motors_e, control_components_s> a,
                  std::pair<tank_motors_e, control_components_s> b) -> bool {
                 return std::abs(a.second.sum()) < std::abs(b.second.sum());
               })
               ->second.sum()) == 1.0,
       "tank velocity balancing with std::map - max velocity");
  bool passed = true;
  for (auto &vel : vels) {
    if (std::abs(vel.second.lin) < std::abs(vel.second.rot)) {
      passed = false;
    }

    if (not rougheq(std::abs(vel.second.rot) /
                        (std::abs(vel.second.lin) + std::abs(vel.second.rot)),
                    pref)) {
      passed = false;
      printf("lin: %f, rot: %f\n", vel.second.lin, vel.second.rot);
    }
  }
  test(passed,
       "tank velocity balancing with std::map - preference to linear ratio");
}

void test_binary_search() {
  std::vector<double> numbers = {-1.0, 1.1, 2.3, 4.3, 9.5};
  test(3 == binary_search(numbers, 4),
       "normal binary search - odd length vector");
  test(0 == binary_search(numbers, -1.1),
       "normal binary search - odd length vector 2");
  numbers = {-1.0, -0.5, 0.1, 0.2, 0.3, 0.4, 4.5};
  test(4 == binary_search(numbers, 0.3),
       "normal binary search - odd length vector 3");
  numbers = {-1.0, 1.1, 2.3, 3.4, 4.3, 9.5};
  test(1 == binary_search(numbers, -0.9),
       "normal binary search - even length vector");
  test(1 == binary_search(numbers, -0.0),
       "normal binary search - even length vector 2");
  test(2 == binary_search(numbers, 2.3),
       "normal binary search - even length vector 3");
  test(6 == binary_search(numbers, 10.0),
       "normal binary search - even length vector 4");
}

void test_binary_search_with_lamda() {
  std::vector<std::pair<double, int>> nums = {
      std::pair<double, int>{2.3, 2},
      std::pair<double, int>{4.2, 4},
      std::pair<double, int>{5.6, 5}
  };
  test(1 == binary_search<std::pair<double, int>, double>(
                nums,
                4.1,
                [](std::pair<double, int> in) -> double { return in.first; }),
       "binary search with lambda converter");
  test(0 == binary_search<std::pair<double, int>, double>(
                nums,
                -1,
                [](std::pair<double, int> in) -> double { return in.first; }),
       "binary search with lambda converter 2");
  test(3 == binary_search<std::pair<double, int>, double>(
                nums,
                6.1,
                [](std::pair<double, int> in) -> double { return in.first; }),
       "binary search with lambda converter 3");
}

int main() {
  test_vel_balance_with_tank();
  test_vel_sum_with_tank();
  test_map_vel_balance_with_tank();
  test_binary_search();
  test_binary_search_with_lamda();
}
