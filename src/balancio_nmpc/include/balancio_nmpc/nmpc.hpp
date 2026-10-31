// Nonlinear MPC for Balancio with acados, the C++ counterpart of
// balancio_control/nmpc.py: same states, inputs, costs and constraints.
//
//   stage state  z = [x; u_prev]   (the previous duty, for the rate limit)
//   input        u = [duty_L, duty_R]
//   dynamics     z+ = [RK4(x, u, dt); u]           (generated C, codegen.hpp)
//   stage cost   e' Q e + u' R u + du' Rd du,  e = x - xref, du = u - u_prev
//   final cost   e' Qf e, Qf from the discrete LQR at the upright equilibrium
//   limits       |u| <= max_duty, |du| <= max_duty_rate dt, |theta| <= max_theta
#pragma once

#include <array>
#include <vector>

#include "balancio_nmpc/model.hpp"

namespace balancio_nmpc {

struct NmpcConfig {
  double dt = 0.02;
  int horizon = 25;
  std::array<double, NX> q{40.0, 100.0, 10.0, 4.0, 0.5, 0.5};
  std::array<double, NU> r{0.1, 0.1};           // duty weights
  std::array<double, NU> r_rate{0.006, 0.006};  // duty-rate weights
  double max_duty = 1.0;
  double max_duty_rate = 300.0;  // [1/s]
  double max_theta = 0.6;        // [rad]
  bool terminal_lqr = true;
  // false: full SQP to convergence. true: SQP-RTI, one QP per solve.
  bool rti = false;
  int max_iter = 100;
  double tol = 1e-6;
};

struct NmpcSolution {
  std::array<double, NU> u{};               // first move, clipped to the duty limit
  std::vector<std::array<double, NX>> x;    // predicted states, N + 1
  std::vector<std::array<double, NU>> u_pred;  // predicted inputs, N
  int status = -1;                          // acados status, 0 = success
  int iterations = 0;
  bool success() const { return status == 0; }
};

// Terminal cost: the discrete Riccati solution at the upright equilibrium.
std::vector<double> dlqr_terminal_cost(const Params &p, const NmpcConfig &c);

class Nmpc {
 public:
  Nmpc(const Params &p, const NmpcConfig &c = {});
  ~Nmpc();
  Nmpc(const Nmpc &) = delete;
  Nmpc &operator=(const Nmpc &) = delete;

  // x0: current state, xref: N + 1 reference states, u_prev: last duty sent.
  NmpcSolution solve(const std::array<double, NX> &x0,
                     const std::vector<std::array<double, NX>> &xref,
                     const std::array<double, NU> &u_prev);

  // Forget the warm start (e.g. after a fall).
  void reset();

  const NmpcConfig &config() const { return cfg_; }

 private:
  struct Impl;
  Impl *impl_;
  NmpcConfig cfg_;
};

}  // namespace balancio_nmpc
