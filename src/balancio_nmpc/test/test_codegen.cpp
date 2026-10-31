// The generated C model functions give the same numbers as CasADi, and their
// Jacobians are laid out the way acados expects.
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "balancio_nmpc/codegen.hpp"
// After CasADi: the generated header #defines casadi_int, which would break
// CasADi's own casadi_int typedef if it came first.
#include "balancio_model.h"

using namespace balancio_nmpc;
using casadi::DM;

namespace {

using GenFn = int (*)(const double **, double **, int *, double *, int);
using WorkFn = int (*)(int *, int *, int *, int *);
using SpFn = const int *(*)(int);

struct Generated {
  GenFn fn;
  WorkFn work;
  SpFn sparsity_out;
};

// Generated sparsity: nrow, ncol, then either 1 (dense) or colind[ncol + 1]
// and row[nnz]. A real colind starts at 0, so a 1 there can only mean dense.
bool is_dense(const int *sp) { return sp[2] == 1; }
int nnz(const int *sp) { return is_dense(sp) ? sp[0] * sp[1] : sp[2 + sp[1]]; }

// Call a generated function; outputs are returned dense (column-major).
std::vector<std::vector<double>> call(const Generated &g, const std::vector<std::vector<double>> &in,
                                      int n_out) {
  int sz_arg, sz_res, sz_iw, sz_w;
  g.work(&sz_arg, &sz_res, &sz_iw, &sz_w);
  std::vector<const double *> arg(sz_arg, nullptr);
  std::vector<double *> res(sz_res, nullptr);
  std::vector<int> iw(sz_iw);
  std::vector<double> w(sz_w);
  for (size_t i = 0; i < in.size(); ++i) arg[i] = in[i].data();
  std::vector<std::vector<double>> nz(n_out), dense(n_out);
  for (int i = 0; i < n_out; ++i) {
    nz[i].resize(nnz(g.sparsity_out(i)));
    res[i] = nz[i].data();
  }
  EXPECT_EQ(g.fn(arg.data(), res.data(), iw.data(), w.data(), 0), 0);
  for (int i = 0; i < n_out; ++i) {
    const int *sp = g.sparsity_out(i);
    const int nrow = sp[0], ncol = sp[1];
    if (is_dense(sp)) {
      dense[i] = nz[i];
      continue;
    }
    const int *colind = sp + 2, *row = sp + 3 + ncol;
    dense[i].assign(nrow * ncol, 0.0);
    for (int c = 0; c < ncol; ++c)
      for (int k = colind[c]; k < colind[c + 1]; ++k) dense[i][row[k] + c * nrow] = nz[i][k];
  }
  return dense;
}

std::vector<double> casadi_eval(const casadi::Function &f, const std::vector<std::vector<double>> &in,
                                int i) {
  std::vector<DM> args;
  for (const auto &v : in) args.push_back(DM(v));
  return DM::densify(f(args)[i]).get_elements();
}

void expect_close(const std::vector<double> &a, const std::vector<double> &b, double tol) {
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i)
    EXPECT_LE(std::fabs(a[i] - b[i]), tol * std::fmax(1.0, std::fabs(b[i]))) << "element " << i;
}

std::vector<double> uniform(std::mt19937 &rng, const std::vector<double> &bound) {
  std::vector<double> v;
  for (double b : bound) v.push_back(std::uniform_real_distribution<double>(-b, b)(rng));
  return v;
}

const Params P = Params::from_yaml(BALANCIO_PARAMS_FILE);
const std::vector<double> X_BOUND{2.0, 0.6, 3.0, 1.0, 5.0, 3.0};
const std::vector<double> U_BOUND{1.0, 1.0};

}  // namespace

TEST(Codegen, MatchesCasadi) {
  const auto fns = make_codegen_functions(P);
  const Generated phi{balancio_dyn_disc_phi_fun, balancio_dyn_disc_phi_fun_work,
                      balancio_dyn_disc_phi_fun_sparsity_out};
  const Generated phi_jac{balancio_dyn_disc_phi_fun_jac, balancio_dyn_disc_phi_fun_jac_work,
                          balancio_dyn_disc_phi_fun_jac_sparsity_out};
  const Generated ekf{balancio_ekf_step, balancio_ekf_step_work, balancio_ekf_step_sparsity_out};
  std::mt19937 rng(0);
  for (int t = 0; t < 200; ++t) {
    auto z = uniform(rng, X_BOUND);
    for (double v : uniform(rng, U_BOUND)) z.push_back(v);
    const auto u = uniform(rng, U_BOUND);
    const std::vector<double> h{std::uniform_real_distribution<double>(0.001, 0.04)(rng)};
    const std::vector<std::vector<double>> in{z, u, h};
    expect_close(call(phi, in, 1)[0], casadi_eval(fns[0], in, 0), 1e-12);
    const auto out = call(phi_jac, in, 2);
    expect_close(out[0], casadi_eval(fns[1], in, 0), 1e-12);
    expect_close(out[1], casadi_eval(fns[1], in, 1), 1e-12);
    const std::vector<double> x(z.begin(), z.begin() + NX);
    const auto e = call(ekf, {x, u, h}, 2);
    expect_close(e[0], casadi_eval(fns[2], {x, u, h}, 0), 1e-12);
    expect_close(e[1], casadi_eval(fns[2], {x, u, h}, 1), 1e-12);
  }
}

TEST(Codegen, DynamicsAreTheModel) {
  // phi = [RK4 step of the model; u], i.e. the same as make_rk4_function.
  const auto F = make_rk4_function(P, 0.02);
  const Generated phi{balancio_dyn_disc_phi_fun, balancio_dyn_disc_phi_fun_work,
                      balancio_dyn_disc_phi_fun_sparsity_out};
  std::mt19937 rng(1);
  for (int t = 0; t < 50; ++t) {
    auto z = uniform(rng, X_BOUND);
    const std::vector<double> x = z;
    for (double v : uniform(rng, U_BOUND)) z.push_back(v);
    const auto u = uniform(rng, U_BOUND);
    const auto out = call(phi, {z, u, {0.02}}, 1)[0];
    auto expected = F(std::vector<DM>{DM(x), DM(u)})[0].get_elements();
    expected.insert(expected.end(), u.begin(), u.end());
    expect_close(out, expected, 1e-12);
  }
}

TEST(Codegen, JacobianLayoutMatchesFiniteDifferences) {
  // acados expects (d phi / d [u; z])': rows are [u; z], columns are phi.
  const Generated phi{balancio_dyn_disc_phi_fun, balancio_dyn_disc_phi_fun_work,
                      balancio_dyn_disc_phi_fun_sparsity_out};
  const Generated phi_jac{balancio_dyn_disc_phi_fun_jac, balancio_dyn_disc_phi_fun_jac_work,
                          balancio_dyn_disc_phi_fun_jac_sparsity_out};
  std::mt19937 rng(2);
  auto z = uniform(rng, X_BOUND);
  for (double v : uniform(rng, U_BOUND)) z.push_back(v);
  const auto u = uniform(rng, U_BOUND);
  const std::vector<double> h{0.02};
  const auto jac_t = call(phi_jac, {z, u, h}, 2)[1];  // (NU + NZ) x NZ, column-major
  constexpr int NV = NU + NZ;
  ASSERT_EQ(jac_t.size(), static_cast<size_t>(NV * NZ));
  const double eps = 1e-6;
  for (int v = 0; v < NV; ++v) {
    auto zp = z, zm = z, up = u, um = u;
    if (v < NU) {
      up[v] += eps;
      um[v] -= eps;
    } else {
      zp[v - NU] += eps;
      zm[v - NU] -= eps;
    }
    const auto fp = call(phi, {zp, up, h}, 1)[0];
    const auto fm = call(phi, {zm, um, h}, 1)[0];
    for (int i = 0; i < NZ; ++i) {
      const double fd = (fp[i] - fm[i]) / (2 * eps);
      EXPECT_NEAR(jac_t[v + i * NV], fd, 1e-6 * std::fmax(1.0, std::fabs(fd)))
          << "d phi[" << i << "] / d [u; z][" << v << "]";
    }
  }
}
