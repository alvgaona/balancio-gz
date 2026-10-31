// The model functions generated as plain C (tools/codegen.cpp), for acados
// and the EKF. Physical parameters are baked in from balancio.yaml at build
// time; the step length h is an input, so one build serves any rate.
//
//   balancio_dyn_disc_phi_fun(z, u, p)      -> phi
//   balancio_dyn_disc_phi_fun_jac(z, u, p)  -> phi, (d phi / d [u; z])'
//       acados discrete dynamics. z = [x; u_prev] (NZ), p = [h],
//       phi = [RK4(x, u, h); u]: carrying the previous duty in the state
//       keeps the duty-rate limit a linear constraint per stage.
//   balancio_ekf_step(x, u, h)              -> x_next, d x_next / d x
#pragma once

#include <casadi/casadi.hpp>
#include <vector>

#include "balancio_nmpc/model.hpp"

namespace balancio_nmpc {

constexpr int NZ = NX + NU;  // NMPC state: x and the previous duty
constexpr int NP = 1;        // acados stage parameter: step length h

// The functions above, as CasADi functions (named as in the generated C).
std::vector<casadi::Function> make_codegen_functions(const Params &p);

// Write them all to <dir>/<name>.c and <dir>/<name>.h.
void generate_c_code(const Params &p, const std::string &dir, const std::string &name);

}  // namespace balancio_nmpc
