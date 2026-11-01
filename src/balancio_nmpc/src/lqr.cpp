// Host-only parts of the NMPC: the LQR terminal cost, computed with CasADi
// from the model. The ESP32 gets the same matrix precomputed.
#include <algorithm>

#include "balancio_nmpc/model.hpp"
#include "balancio_nmpc/nmpc.hpp"

namespace balancio_nmpc {

std::vector<double> dlqr_terminal_cost(const Params &p, const NmpcConfig &c) {
  using casadi::DM;
  using casadi::SX;
  const SX x = SX::sym("x", NX), u = SX::sym("u", NU);
  const SX xn = rk4_step(x, u, c.dt, p);
  casadi::Function lin("lin", {x, u}, {SX::jacobian(xn, x), SX::jacobian(xn, u)});
  const auto AB = lin(std::vector<DM>{DM::zeros(NX), DM::zeros(NU)});
  const DM A = AB[0], B = AB[1];
  const DM Q = DM::diag(DM(std::vector<double>(c.q.begin(), c.q.end())));
  const DM R = DM::diag(DM(std::vector<double>(c.r.begin(), c.r.end())));
  // Same fixed-point iteration as balancio_control/nmpc.py.
  DM P = Q;
  for (int i = 0; i < 2000; ++i) {
    const DM BtP = DM::mtimes(B.T(), P);
    const DM K = DM::solve(R + DM::mtimes(BtP, B), DM::mtimes(BtP, A));
    const DM Pn = Q + DM::mtimes({A.T(), P, A}) - DM::mtimes({A.T(), P, B, K});
    const double diff = static_cast<double>(DM::norm_inf(Pn - P));
    const double scale = std::max(1.0, static_cast<double>(DM::norm_inf(P)));
    P = Pn;
    if (diff < 1e-9 * scale) break;
  }
  P = 0.5 * (P + P.T());
  return P.get_elements();  // column-major NX x NX
}

namespace {
NmpcConfig with_terminal_cost(const Params &p, NmpcConfig c) {
  if (c.terminal_lqr && c.qf.empty()) c.qf = dlqr_terminal_cost(p, c);
  return c;
}
}  // namespace

Nmpc::Nmpc(const Params &p, const NmpcConfig &c) : Nmpc(with_terminal_cost(p, c)) {}

}  // namespace balancio_nmpc
