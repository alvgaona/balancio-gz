// The C++ model matches balancio_control/model.py, and behaves physically.
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "balancio_nmpc/model.hpp"

using balancio_nmpc::NU;
using balancio_nmpc::NX;
using balancio_nmpc::Params;
using casadi::DM;

namespace {

constexpr double DT = 0.02;  // as in make_model_reference.py

Params params() { return Params::from_yaml(BALANCIO_PARAMS_FILE); }

std::vector<double> eval(const casadi::Function &fn, const std::vector<double> &x,
                         const std::vector<double> &u) {
  return fn(std::vector<DM>{DM(x), DM(u)})[0].get_elements();
}

std::vector<std::vector<double>> load_reference() {
  std::ifstream in(MODEL_REFERENCE);
  std::vector<std::vector<double>> rows;
  std::string line;
  while (std::getline(in, line)) {
    std::vector<double> row;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) row.push_back(std::stod(cell));
    rows.push_back(row);
  }
  return rows;
}

// |a - b| <= tol * max(1, |b|)
void expect_close(const std::vector<double> &a, const double *b, int n, double tol,
                  const std::string &what) {
  for (int i = 0; i < n; ++i)
    EXPECT_LE(std::fabs(a[i] - b[i]), tol * std::fmax(1.0, std::fabs(b[i])))
        << what << "[" << i << "]: " << a[i] << " vs " << b[i];
}

}  // namespace

TEST(Model, MatchesPython) {
  const Params p = params();
  const auto f = balancio_nmpc::make_dynamics_function(p);
  const auto F = balancio_nmpc::make_rk4_function(p, DT);
  const auto rows = load_reference();
  ASSERT_GE(rows.size(), 100u) << "reference file missing or short";
  for (const auto &row : rows) {
    ASSERT_EQ(row.size(), 20u);
    const std::vector<double> x(row.begin(), row.begin() + NX);
    const std::vector<double> u(row.begin() + NX, row.begin() + NX + NU);
    expect_close(eval(f, x, u), &row[8], NX, 1e-12, "f");
    expect_close(eval(F, x, u), &row[14], NX, 1e-12, "F");
  }
}

TEST(Model, UprightIsEquilibriumAndUnstable) {
  const auto f = balancio_nmpc::make_dynamics_function(params());
  for (double v : eval(f, std::vector<double>(NX, 0.0), {0.0, 0.0})) EXPECT_EQ(v, 0.0);
  // Leaning forward with no duty accelerates the fall.
  EXPECT_GT(eval(f, {0, 0.1, 0, 0, 0, 0}, {0.0, 0.0})[4], 0.0);
}

TEST(Model, DutySigns) {
  const auto f = balancio_nmpc::make_dynamics_function(params());
  const auto fwd = eval(f, std::vector<double>(NX, 0.0), {0.5, 0.5});
  EXPECT_GT(fwd[3], 0.0);  // wheels forward
  EXPECT_LT(fwd[4], 0.0);  // body pitched back
  const auto turn = eval(f, std::vector<double>(NX, 0.0), {-0.5, 0.5});
  EXPECT_GT(turn[5], 0.0);  // right wheel forward -> yaw left (CCW)
}

TEST(Model, BackEmfLimitsSpeed) {
  // Full duty, held upright: back-EMF and axle friction balance the motor.
  const Params p = params();
  const auto F = balancio_nmpc::make_rk4_function(p, 0.001);
  std::vector<double> x(NX, 0.0);
  for (int i = 0; i < 3000; ++i) {
    x = eval(F, x, {1.0, 1.0});
    x[1] = x[4] = 0.0;  // hold the body upright
  }
  const double k = p.max_torque / p.motor_no_load_speed;
  EXPECT_NEAR(x[3] / p.r, p.max_torque / (k + p.calpha), 0.01 * p.max_torque / (k + p.calpha));
}
