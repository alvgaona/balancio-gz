// The toolchain works: CasADi C++ builds a model and its Jacobians, and
// acados + HPIPM solve an MPC QP for it with the same answer as IPOPT.
#include <gtest/gtest.h>

#include <casadi/casadi.hpp>
#include <vector>

extern "C" {
#include "acados_c/ocp_qp_interface.h"
}

namespace {

constexpr int N = 20;
constexpr int NX = 2;
constexpr int NU = 1;
constexpr double DT = 0.1;
constexpr double R_WEIGHT = 0.1;

// Double integrator, linearized by CasADi: returns A, B (column-major).
std::vector<casadi::DM> double_integrator() {
  using casadi::SX;
  SX x = SX::sym("x", NX), u = SX::sym("u", NU);
  SX xn = SX::vertcat({x(0) + DT * x(1) + 0.5 * DT * DT * u(0), x(1) + DT * u(0)});
  casadi::Function lin("lin", {x, u}, {SX::jacobian(xn, x), SX::jacobian(xn, u)});
  return lin(std::vector<casadi::DM>{casadi::DM::zeros(NX), casadi::DM::zeros(NU)});
}

std::vector<double> solve_hpipm(const casadi::DM &Ad, const casadi::DM &Bd,
                                const std::vector<double> &x0, int *status) {
  std::vector<double> A = Ad.get_elements(), B = Bd.get_elements();
  ocp_qp_solver_plan_t plan;
  plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
  auto *config = ocp_qp_xcond_solver_config_create(plan);
  auto *dims = ocp_qp_xcond_solver_dims_create(config, N);
  int nx = NX, nu = NU, zero = 0;
  for (int k = 0; k <= N; ++k) {
    ocp_qp_xcond_solver_dims_set(config, dims, k, "nx", &nx);
    ocp_qp_xcond_solver_dims_set(config, dims, k, "nu", k < N ? &nu : &zero);
    ocp_qp_xcond_solver_dims_set(config, dims, k, "nbx", k == 0 ? &nx : &zero);
    ocp_qp_xcond_solver_dims_set(config, dims, k, "nbu", k < N ? &nu : &zero);
  }
  auto *in = ocp_qp_in_create_from_xcond_dims(dims);
  auto *out = ocp_qp_out_create_from_xcond_dims(dims);

  double Q[] = {1, 0, 0, 1}, R[] = {R_WEIGHT}, b[] = {0, 0};
  double lbu[] = {-1.0}, ubu[] = {1.0};
  int idxbx[] = {0, 1}, idxbu[] = {0};
  std::vector<double> xb = x0;
  for (int k = 0; k <= N; ++k) {
    ocp_qp_in_set(config, in, k, const_cast<char *>("Q"), Q);
    if (k < N) {
      ocp_qp_in_set(config, in, k, const_cast<char *>("A"), A.data());
      ocp_qp_in_set(config, in, k, const_cast<char *>("B"), B.data());
      ocp_qp_in_set(config, in, k, const_cast<char *>("b"), b);
      ocp_qp_in_set(config, in, k, const_cast<char *>("R"), R);
      ocp_qp_in_set(config, in, k, const_cast<char *>("idxbu"), idxbu);
      ocp_qp_in_set(config, in, k, const_cast<char *>("lbu"), lbu);
      ocp_qp_in_set(config, in, k, const_cast<char *>("ubu"), ubu);
    }
  }
  ocp_qp_in_set(config, in, 0, const_cast<char *>("idxbx"), idxbx);
  ocp_qp_in_set(config, in, 0, const_cast<char *>("lbx"), xb.data());
  ocp_qp_in_set(config, in, 0, const_cast<char *>("ubx"), xb.data());

  void *opts = ocp_qp_xcond_solver_opts_create(config, dims);
  auto *solver = ocp_qp_create(config, dims, opts);
  *status = ocp_qp_solve(solver, in, out);
  std::vector<double> u(N);
  for (int k = 0; k < N; ++k) ocp_qp_out_get(out, k, "u", &u[k]);

  ocp_qp_solver_destroy(solver);
  ocp_qp_xcond_solver_opts_free(static_cast<ocp_qp_xcond_solver_opts *>(opts));
  ocp_qp_in_free(in);
  ocp_qp_out_free(out);
  ocp_qp_xcond_solver_dims_free(dims);
  ocp_qp_xcond_solver_config_free(config);
  return u;
}

std::vector<double> solve_ipopt(const casadi::DM &Ad, const casadi::DM &Bd,
                                const std::vector<double> &x0) {
  using casadi::MX;
  using casadi::Slice;
  casadi::Opti opti;
  MX X = opti.variable(NX, N + 1), U = opti.variable(NU, N);
  MX J = MX::sumsqr(X(Slice(), N));
  for (int k = 0; k < N; ++k) {
    J += MX::sumsqr(X(Slice(), k)) + R_WEIGHT * MX::sumsqr(U(Slice(), k));
    opti.subject_to(X(Slice(), k + 1) ==
                    MX::mtimes(Ad, X(Slice(), k)) + MX::mtimes(Bd, U(Slice(), k)));
  }
  opti.subject_to(X(Slice(), 0) == casadi::DM(x0));
  opti.subject_to(opti.bounded(-1, U, 1));
  opti.minimize(0.5 * J);  // acados uses 1/2 x'Qx
  opti.solver("ipopt", {{"print_time", false}},
              {{"print_level", 0}, {"sb", "yes"}, {"tol", 1e-10}});
  return opti.solve().value(U).get_elements();
}

}  // namespace

TEST(Toolchain, HpipmMatchesIpopt) {
  auto AB = double_integrator();
  const std::vector<double> x0{1.0, 0.0};
  int status = -1;
  auto u = solve_hpipm(AB[0], AB[1], x0, &status);
  auto u_ref = solve_ipopt(AB[0], AB[1], x0);
  ASSERT_EQ(status, 0);
  EXPECT_NEAR(u[0], -1.0, 1e-8) << "first move should be on the input limit";
  for (int k = 0; k < N; ++k) EXPECT_NEAR(u[k], u_ref[k], 1e-6) << "stage " << k;
}
