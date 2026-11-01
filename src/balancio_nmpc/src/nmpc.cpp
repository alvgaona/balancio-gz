#include "balancio_nmpc/nmpc.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "balancio_nmpc/dims.hpp"

extern "C" {
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"
#include "balancio_model.h"
}

namespace balancio_nmpc {

namespace {

constexpr int NY = NX + NU + NU;  // stage residual: [x; u; u - u_prev]

// acados declares the last argument of a CasADi function as void *mem; the
// generated code has int mem. acados always passes 0/NULL there, and both
// travel in the same register on the targets we build for.
using AcadosCasadiFun = int (*)(const double **, double **, int *, double *, void *);

void map_casadi(external_function_external_param_casadi &f,
                int (*fun)(const casadi_real **, casadi_real **, casadi_int *, casadi_real *, int),
                int (*work)(casadi_int *, casadi_int *, casadi_int *, casadi_int *),
                const casadi_int *(*sp_in)(casadi_int), const casadi_int *(*sp_out)(casadi_int),
                casadi_int (*n_in)(void), casadi_int (*n_out)(void),
                external_function_opts &opts) {
  // Through a generic function pointer: the documented way to say this
  // signature change is intended (no -Wcast-function-type warning).
  f.casadi_fun = reinterpret_cast<AcadosCasadiFun>(reinterpret_cast<void (*)()>(fun));
  f.casadi_work = work;
  f.casadi_sparsity_in = sp_in;
  f.casadi_sparsity_out = sp_out;
  f.casadi_n_in = n_in;
  f.casadi_n_out = n_out;
  external_function_external_param_casadi_create(&f, &opts);
}

// Column-major dense matrix helpers.
std::vector<double> zeros(int rows, int cols) { return std::vector<double>(rows * cols, 0.0); }
void put(std::vector<double> &m, int rows, int i, int j, double v) { m[i + j * rows] = v; }

}  // namespace

struct Nmpc::Impl {
  int N;
  ocp_nlp_plan_t *plan = nullptr;
  ocp_nlp_config *config = nullptr;
  ocp_nlp_dims *dims = nullptr;
  ocp_nlp_in *in = nullptr;
  ocp_nlp_out *out = nullptr;
  void *opts = nullptr;
  ocp_nlp_solver *solver = nullptr;
  std::vector<external_function_external_param_casadi> phi, phi_jac;
  bool warm = false;
};

Nmpc::Nmpc(const NmpcConfig &c) : impl_(new Impl), cfg_(c) {
  Impl &m = *impl_;
  const int N = m.N = c.horizon;

  // ---- plan
  m.plan = ocp_nlp_plan_create(N);
  m.plan->nlp_solver = c.rti ? SQP_RTI : SQP;
  m.plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
  m.plan->relaxed_ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
  for (int k = 0; k <= N; ++k) {
    m.plan->nlp_cost[k] = LINEAR_LS;
    m.plan->nlp_constraints[k] = BGH;
  }
  for (int k = 0; k < N; ++k) {
    m.plan->nlp_dynamics[k] = DISCRETE_MODEL;
    m.plan->sim_solver_plan[k].sim_solver = INVALID_SIM_SOLVER;
  }
  m.plan->regularization = NO_REGULARIZE;
  m.plan->globalization = FIXED_STEP;
  m.config = ocp_nlp_config_create(*m.plan);

  // ---- dimensions
  m.dims = ocp_nlp_dims_create(m.config);
  std::vector<int> nx(N + 1, NZ), nu(N + 1, NU), nz(N + 1, 0), ns(N + 1, 0), np(N + 1, NP);
  nu[N] = 0;
  ocp_nlp_dims_set_opt_vars(m.config, m.dims, "nx", nx.data());
  ocp_nlp_dims_set_opt_vars(m.config, m.dims, "nu", nu.data());
  ocp_nlp_dims_set_opt_vars(m.config, m.dims, "nz", nz.data());
  ocp_nlp_dims_set_opt_vars(m.config, m.dims, "ns", ns.data());
  ocp_nlp_dims_set_opt_vars(m.config, m.dims, "np", np.data());
  int zero = 0, one = 1, nzv = NZ, nuv = NU, ny = NY, ny_e = NX;
  for (int k = 0; k <= N; ++k) {
    int *nbx = k == 0 ? &nzv : &one;  // stage 0: z fixed; others: pitch
    ocp_nlp_dims_set_constraints(m.config, m.dims, k, "nbx", nbx);
    ocp_nlp_dims_set_constraints(m.config, m.dims, k, "nbxe", k == 0 ? &nzv : &zero);
    ocp_nlp_dims_set_constraints(m.config, m.dims, k, "nbu", k < N ? &nuv : &zero);
    ocp_nlp_dims_set_constraints(m.config, m.dims, k, "ng", k < N ? &nuv : &zero);
    for (const char *f : {"nsbx", "nsbu", "nsg", "nh", "nsh"})
      ocp_nlp_dims_set_constraints(m.config, m.dims, k, f, &zero);
    ocp_nlp_dims_set_cost(m.config, m.dims, k, "ny", k < N ? &ny : &ny_e);
  }

  // ---- model functions (generated C)
  m.in = ocp_nlp_in_create(m.config, m.dims);
  external_function_opts ext_opts;
  external_function_opts_set_to_default(&ext_opts);
  m.phi.resize(N);
  m.phi_jac.resize(N);
  for (int k = 0; k < N; ++k) {
    map_casadi(m.phi[k], balancio_dyn_disc_phi_fun, balancio_dyn_disc_phi_fun_work,
               balancio_dyn_disc_phi_fun_sparsity_in, balancio_dyn_disc_phi_fun_sparsity_out,
               balancio_dyn_disc_phi_fun_n_in, balancio_dyn_disc_phi_fun_n_out, ext_opts);
    map_casadi(m.phi_jac[k], balancio_dyn_disc_phi_fun_jac, balancio_dyn_disc_phi_fun_jac_work,
               balancio_dyn_disc_phi_fun_jac_sparsity_in, balancio_dyn_disc_phi_fun_jac_sparsity_out,
               balancio_dyn_disc_phi_fun_jac_n_in, balancio_dyn_disc_phi_fun_jac_n_out, ext_opts);
    ocp_nlp_dynamics_model_set_external_param_fun(m.config, m.dims, m.in, k, "disc_dyn_fun",
                                                  &m.phi[k]);
    ocp_nlp_dynamics_model_set_external_param_fun(m.config, m.dims, m.in, k, "disc_dyn_fun_jac",
                                                  &m.phi_jac[k]);
  }

  // ---- numbers: step length, costs, constraints
  double dt = c.dt;
  for (int k = 0; k <= N; ++k) {
    ocp_nlp_in_set(m.config, m.dims, m.in, k, "Ts", &dt);
    ocp_nlp_in_set(m.config, m.dims, m.in, k, "parameter_values", &dt);
  }

  // y = Vx z + Vu u = [x; u; u - u_prev]; W = 2 blkdiag(Q, R, Rd), since acados
  // minimizes 1/2 |y - yref|^2_W and nmpc.py has no 1/2.
  auto Vx = zeros(NY, NZ), Vu = zeros(NY, NU), W = zeros(NY, NY);
  for (int i = 0; i < NX; ++i) {
    put(Vx, NY, i, i, 1.0);
    put(W, NY, i, i, 2 * c.q[i]);
  }
  for (int i = 0; i < NU; ++i) {
    put(Vu, NY, NX + i, i, 1.0);
    put(W, NY, NX + i, NX + i, 2 * c.r[i]);
    put(Vu, NY, NX + NU + i, i, 1.0);
    put(Vx, NY, NX + NU + i, NX + i, -1.0);
    put(W, NY, NX + NU + i, NX + NU + i, 2 * c.r_rate[i]);
  }
  std::vector<double> yref(NY, 0.0);
  for (int k = 0; k < N; ++k) {
    ocp_nlp_cost_model_set(m.config, m.dims, m.in, k, "Vx", Vx.data());
    ocp_nlp_cost_model_set(m.config, m.dims, m.in, k, "Vu", Vu.data());
    ocp_nlp_cost_model_set(m.config, m.dims, m.in, k, "W", W.data());
    ocp_nlp_cost_model_set(m.config, m.dims, m.in, k, "yref", yref.data());
  }
  auto Vx_e = zeros(NX, NZ);
  for (int i = 0; i < NX; ++i) put(Vx_e, NX, i, i, 1.0);
  std::vector<double> W_e;
  if (!c.qf.empty()) {
    if (c.qf.size() != NX * NX) throw std::invalid_argument("NmpcConfig::qf: expected NX x NX");
    W_e = c.qf;
  } else {
    W_e = zeros(NX, NX);
    for (int i = 0; i < NX; ++i) put(W_e, NX, i, i, c.q[i]);
  }
  for (double &v : W_e) v *= 2;
  ocp_nlp_cost_model_set(m.config, m.dims, m.in, N, "Vx", Vx_e.data());
  ocp_nlp_cost_model_set(m.config, m.dims, m.in, N, "W", W_e.data());
  ocp_nlp_cost_model_set(m.config, m.dims, m.in, N, "yref", yref.data());

  m.out = ocp_nlp_out_create(m.config, m.dims);

  // Stage 0: z fixed to [x0; u_prev] (values set in solve).
  std::vector<int> idx_z(NZ);
  for (int i = 0; i < NZ; ++i) idx_z[i] = i;
  std::vector<double> z0(NZ, 0.0);
  ocp_nlp_constraints_model_set(m.config, m.dims, m.in, m.out, 0, "idxbx", idx_z.data());
  ocp_nlp_constraints_model_set(m.config, m.dims, m.in, m.out, 0, "lbx", z0.data());
  ocp_nlp_constraints_model_set(m.config, m.dims, m.in, m.out, 0, "ubx", z0.data());
  ocp_nlp_constraints_model_set(m.config, m.dims, m.in, m.out, 0, "idxbxe", idx_z.data());
  // Stages 1..N: pitch limit.
  int idx_theta = 1;
  double lth = -c.max_theta, uth = c.max_theta;
  for (int k = 1; k <= N; ++k) {
    ocp_nlp_constraints_model_set(m.config, m.dims, m.in, m.out, k, "idxbx", &idx_theta);
    ocp_nlp_constraints_model_set(m.config, m.dims, m.in, m.out, k, "lbx", &lth);
    ocp_nlp_constraints_model_set(m.config, m.dims, m.in, m.out, k, "ubx", &uth);
  }
  // Duty limit (box) and duty-rate limit: -du_max <= u - u_prev <= du_max.
  int idx_u[NU] = {0, 1};
  double lbu[NU] = {-c.max_duty, -c.max_duty}, ubu[NU] = {c.max_duty, c.max_duty};
  const double du_max = c.max_duty_rate * c.dt;
  double lg[NU] = {-du_max, -du_max}, ug[NU] = {du_max, du_max};
  auto C = zeros(NU, NZ), D = zeros(NU, NU);
  for (int i = 0; i < NU; ++i) {
    put(C, NU, i, NX + i, -1.0);
    put(D, NU, i, i, 1.0);
  }
  for (int k = 0; k < N; ++k) {
    ocp_nlp_constraints_model_set(m.config, m.dims, m.in, m.out, k, "idxbu", idx_u);
    ocp_nlp_constraints_model_set(m.config, m.dims, m.in, m.out, k, "lbu", lbu);
    ocp_nlp_constraints_model_set(m.config, m.dims, m.in, m.out, k, "ubu", ubu);
    ocp_nlp_constraints_model_set(m.config, m.dims, m.in, m.out, k, "C", C.data());
    ocp_nlp_constraints_model_set(m.config, m.dims, m.in, m.out, k, "D", D.data());
    ocp_nlp_constraints_model_set(m.config, m.dims, m.in, m.out, k, "lg", lg);
    ocp_nlp_constraints_model_set(m.config, m.dims, m.in, m.out, k, "ug", ug);
  }

  // ---- solver options
  m.opts = ocp_nlp_solver_opts_create(m.config, m.dims);
  int max_iter = c.max_iter, warm = 1, print = 0, iter_max_qp = 100;
  double tol = c.tol, step = 1.0;
  ocp_nlp_solver_opts_set(m.config, m.opts, "globalization_fixed_step_length", &step);
  if (!c.rti) {
    ocp_nlp_solver_opts_set(m.config, m.opts, "max_iter", &max_iter);
    for (const char *f : {"tol_stat", "tol_eq", "tol_ineq", "tol_comp"})
      ocp_nlp_solver_opts_set(m.config, m.opts, f, &tol);
  }
  ocp_nlp_solver_opts_set(m.config, m.opts, "qp_warm_start", &warm);
  ocp_nlp_solver_opts_set(m.config, m.opts, "qp_iter_max", &iter_max_qp);
  ocp_nlp_solver_opts_set(m.config, m.opts, "print_level", &print);

  m.solver = ocp_nlp_solver_create(m.config, m.dims, m.opts, m.in);
  if (ocp_nlp_precompute(m.solver, m.in, m.out) != 0)
    throw std::runtime_error("acados: ocp_nlp_precompute failed");
}

Nmpc::~Nmpc() {
  Impl &m = *impl_;
  ocp_nlp_solver_destroy(m.solver);
  ocp_nlp_out_destroy(m.out);
  ocp_nlp_in_destroy(m.in);
  ocp_nlp_solver_opts_destroy(m.opts);
  ocp_nlp_dims_destroy(m.dims);
  ocp_nlp_config_destroy(m.config);
  ocp_nlp_plan_destroy(m.plan);
  for (auto &f : m.phi) external_function_external_param_casadi_free(&f);
  for (auto &f : m.phi_jac) external_function_external_param_casadi_free(&f);
  delete impl_;
}

void Nmpc::reset() { impl_->warm = false; }

NmpcSolution Nmpc::solve(const std::array<double, NX> &x0,
                         const std::vector<std::array<double, NX>> &xref,
                         const std::array<double, NU> &u_prev) {
  Impl &m = *impl_;
  const int N = m.N;
  if (static_cast<int>(xref.size()) != N + 1)
    throw std::invalid_argument("Nmpc::solve: xref needs horizon + 1 states");

  double z0[NZ];
  std::copy(x0.begin(), x0.end(), z0);
  std::copy(u_prev.begin(), u_prev.end(), z0 + NX);
  ocp_nlp_constraints_model_set(m.config, m.dims, m.in, m.out, 0, "lbx", z0);
  ocp_nlp_constraints_model_set(m.config, m.dims, m.in, m.out, 0, "ubx", z0);

  for (int k = 0; k <= N; ++k) {
    std::vector<double> y(k < N ? NY : NX, 0.0);
    std::copy(xref[k].begin(), xref[k].end(), y.begin());
    ocp_nlp_cost_model_set(m.config, m.dims, m.in, k, "yref", y.data());
  }

  // Initial guess: the previous solution shifted by one stage, or z0.
  if (m.warm) {
    std::vector<double> zk(NZ), uk(NU);
    for (int k = 0; k < N; ++k) {
      ocp_nlp_out_get(m.config, m.dims, m.out, k + 1, "x", zk.data());
      ocp_nlp_out_set(m.config, m.dims, m.out, m.in, k, "x", zk.data());
      if (k + 1 < N) {
        ocp_nlp_out_get(m.config, m.dims, m.out, k + 1, "u", uk.data());
        ocp_nlp_out_set(m.config, m.dims, m.out, m.in, k, "u", uk.data());
      }
    }
  } else {
    ocp_nlp_out_set_values_to_zero(m.config, m.dims, m.out);
    std::vector<double> u0(u_prev.begin(), u_prev.end());
    for (int k = 0; k <= N; ++k) {
      ocp_nlp_out_set(m.config, m.dims, m.out, m.in, k, "x", z0);
      if (k < N) ocp_nlp_out_set(m.config, m.dims, m.out, m.in, k, "u", u0.data());
    }
  }
  ocp_nlp_out_set(m.config, m.dims, m.out, m.in, 0, "x", z0);

  NmpcSolution sol;
  sol.status = ocp_nlp_solve(m.solver, m.in, m.out);
  ocp_nlp_get(m.solver, "sqp_iter", &sol.iterations);
  m.warm = sol.success();

  sol.x.resize(N + 1);
  sol.u_pred.resize(N);
  double zk[NZ];
  for (int k = 0; k <= N; ++k) {
    ocp_nlp_out_get(m.config, m.dims, m.out, k, "x", zk);
    std::copy(zk, zk + NX, sol.x[k].begin());
    if (k < N) ocp_nlp_out_get(m.config, m.dims, m.out, k, "u", sol.u_pred[k].data());
  }
  for (int i = 0; i < NU; ++i) {
    const double u = std::isfinite(sol.u_pred[0][i]) ? sol.u_pred[0][i] : 0.0;
    sol.u[i] = std::clamp(u, -cfg_.max_duty, cfg_.max_duty);
  }
  return sol;
}

}  // namespace balancio_nmpc
