// The acados NMPC matches balancio_control/nmpc.py and balances the robot.
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "balancio_nmpc/model.hpp"
#include "balancio_nmpc/nmpc.hpp"

using namespace balancio_nmpc;
using casadi::DM;
using State = std::array<double, NX>;
using Input = std::array<double, NU>;

namespace {

const Params P = Params::from_yaml(BALANCIO_PARAMS_FILE);

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

// Reference as mpc_node.py builds it: x, psi integrated from (v, w).
std::vector<State> reference(int N, double dt, double x0, double psi0, double v, double w) {
  std::vector<State> ref(N + 1);
  for (int k = 0; k <= N; ++k) ref[k] = {x0 + v * k * dt, 0.0, psi0 + w * k * dt, v, 0.0, w};
  return ref;
}

// +-10 % on every parameter, as in test_nmpc.py (fields in TwsbrParams order).
Params mismatched(Params p) {
  double *f[] = {&p.d, &p.l, &p.r, &p.mb, &p.mw, &p.J, &p.K, &p.I1, &p.I2, &p.I3,
                 &p.calpha, &p.max_torque, &p.motor_no_load_speed, &p.motor_inertia};
  for (int i = 0; i < 14; ++i) *f[i] *= (i % 2) ? 1.1 : 0.9;
  return p;
}

State step(const casadi::Function &plant, const State &x, const Input &u) {
  const auto xn = plant(std::vector<DM>{DM(std::vector<double>(x.begin(), x.end())),
                                        DM(std::vector<double>(u.begin(), u.end()))})[0]
                      .get_elements();
  State out;
  std::copy(xn.begin(), xn.end(), out.begin());
  return out;
}

// test_closed_loop_recovery_and_tracking from test_nmpc.py.
void closed_loop(const NmpcConfig &cfg, double settle_tol) {
  Nmpc mpc(P, cfg);
  const auto plant = make_rk4_function(mismatched(P), cfg.dt, 20);
  State x{0.0, 0.3, 0.0, 0.0, 0.0, 0.0};
  Input u{0.0, 0.0};
  double x_ref = 0.0, psi_ref = 0.0;
  for (int i = 0; i < static_cast<int>(8.0 / cfg.dt); ++i) {
    const double t = i * cfg.dt;
    const double v = t > 3.0 ? 0.3 : 0.0, w = t > 3.0 ? 0.5 : 0.0;
    const auto sol = mpc.solve(x, reference(cfg.horizon, cfg.dt, x_ref, psi_ref, v, w), u);
    ASSERT_TRUE(sol.success()) << "status " << sol.status << " at t = " << t;
    for (double ui : sol.u) ASSERT_LE(std::fabs(ui), cfg.max_duty + 1e-9);
    u = sol.u;
    x = step(plant, x, u);
    x_ref += v * cfg.dt;
    psi_ref += w * cfg.dt;
    ASSERT_LT(std::fabs(x[1]), cfg.max_theta + 0.05) << "t = " << t;
    if (std::fabs(t - 3.0) < 1e-9) {
      EXPECT_LT(std::fabs(x[1]), settle_tol) << "not balanced at 3 s";
      EXPECT_LT(std::fabs(x[0]), 0.05) << "drifted by 3 s";
    }
  }
  EXPECT_NEAR(x[3], 0.3, 0.05);
  EXPECT_NEAR(x[5], 0.5, 0.05);
  EXPECT_LT(std::fabs(x[1]), 0.1);
}

}  // namespace

TEST(Nmpc, MatchesPython) {
  NmpcConfig cfg;
  cfg.tol = 1e-10;  // the reference is solved tightly too
  Nmpc mpc(P, cfg);
  const auto rows = load_csv(NMPC_REFERENCE);
  ASSERT_GE(rows.size(), 20u) << "reference file missing or short";
  double worst = 0.0;
  for (const auto &r : rows) {
    ASSERT_EQ(r.size(), 12u);
    const State x0{r[0], r[1], r[2], r[3], r[4], r[5]};
    const Input u_prev{r[6], r[7]};
    mpc.reset();
    const auto sol = mpc.solve(x0, reference(cfg.horizon, cfg.dt, 0.0, r[2], r[8], r[9]), u_prev);
    ASSERT_TRUE(sol.success()) << "status " << sol.status;
    for (int i = 0; i < NU; ++i) {
      EXPECT_NEAR(sol.u[i], r[10 + i], 1e-6);
      worst = std::fmax(worst, std::fabs(sol.u[i] - r[10 + i]));
    }
  }
  std::printf("max |u0 - u0_python| = %.2e over %zu cases\n", worst, rows.size());
}

TEST(Nmpc, ClosedLoopSqp) { closed_loop(NmpcConfig{}, 0.02); }

TEST(Nmpc, ClosedLoopRti) {
  NmpcConfig cfg;
  cfg.rti = true;
  closed_loop(cfg, 0.02);
}
