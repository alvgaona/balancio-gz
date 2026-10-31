// The C++ ImuEkf replays a run of balancio_control/estimator.py and gives
// the same estimates.
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "balancio_nmpc/ekf.hpp"

using namespace balancio_nmpc;

namespace {

std::vector<std::vector<double>> load_csv(const char *path) {
  std::ifstream in(path);
  std::vector<std::vector<double>> rows;
  std::string line, cell;
  while (std::getline(in, line)) {
    std::vector<double> row;
    std::stringstream ss(line);
    while (std::getline(ss, cell, ',')) row.push_back(std::stod(cell));
    rows.push_back(row);
  }
  return rows;
}

}  // namespace

TEST(Ekf, WrapAndEuler) {
  EXPECT_NEAR(wrap_angle(3.5), 3.5 - 2 * M_PI, 1e-15);
  EXPECT_NEAR(wrap_angle(-3.5), 2 * M_PI - 3.5, 1e-15);
  // 0.3 rad about y, then 1.0 rad about z: q = qz(1.0) * qy(0.3)
  const double c1 = std::cos(0.5), s1 = std::sin(0.5), c2 = std::cos(0.15), s2 = std::sin(0.15);
  const auto e = quat_to_euler(-s1 * s2, c1 * s2, s1 * c2, c1 * c2);
  EXPECT_NEAR(e[0], 0.0, 1e-12);
  EXPECT_NEAR(e[1], 0.3, 1e-12);
  EXPECT_NEAR(e[2], 1.0, 1e-12);
}

TEST(Ekf, MatchesPython) {
  const auto rows = load_csv(EKF_REFERENCE);
  ASSERT_GT(rows.size(), 1000u);
  ImuEkf ekf(0.02);
  int steps = 0;
  double worst = 0.0, worst_xy = 0.0;
  for (const auto &r : rows) {
    switch (static_cast<int>(r[0])) {
      case 0:
        ekf.update_imu({r[2], r[3], r[4], r[5]}, {r[6], r[7], r[8]}, r[1]);
        break;
      case 1:
        ekf.set_input(r[1], {r[2], r[3]});
        break;
      case 2: {
        const State x = ekf.step(r[1]);
        for (int i = 0; i < NX; ++i) worst = std::max(worst, std::fabs(x[i] - r[2 + i]));
        worst_xy = std::max({worst_xy, std::fabs(ekf.X() - r[8]), std::fabs(ekf.Y() - r[9])});
        ++steps;
        break;
      }
      case 3:
        ekf.reset();
        break;
    }
  }
  EXPECT_GT(steps, 250);
  EXPECT_LT(worst, 1e-9);
  EXPECT_LT(worst_xy, 1e-9);
  std::cout << steps << " steps, max state error " << worst << ", max X/Y error " << worst_xy
            << "\n";
}
