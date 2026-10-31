#include "balancio_nmpc/codegen.hpp"

namespace balancio_nmpc {

using casadi::SX;

std::vector<casadi::Function> make_codegen_functions(const Params &p) {
  // Common subexpression elimination, as acados does.
  const casadi::Dict opts{{"cse", true}};

  const SX z = SX::sym("z", NZ), u = SX::sym("u", NU), prm = SX::sym("p", NP);
  const SX x = z(casadi::Slice(0, NX));
  const SX phi = SX::vertcat({rk4_step(x, u, prm(0), p), u});
  const SX jac_ux = SX::jacobian(phi, SX::vertcat({u, z}));

  const SX xe = SX::sym("x", NX), ue = SX::sym("u", NU), h = SX::sym("h");
  const SX xn = rk4_step(xe, ue, h, p);

  return {
      casadi::Function("balancio_dyn_disc_phi_fun", {z, u, prm}, {phi}, opts),
      casadi::Function("balancio_dyn_disc_phi_fun_jac", {z, u, prm}, {phi, jac_ux.T()}, opts),
      casadi::Function("balancio_ekf_step", {xe, ue, h}, {xn, SX::jacobian(xn, xe)}, opts),
  };
}

void generate_c_code(const Params &p, const std::string &dir, const std::string &name) {
  // The options acados uses for its own model functions. std::string, not
  // bare literals: a const char* would convert to bool.
  casadi::CodeGenerator gen(name + ".c", {{"with_header", true},
                                          {"casadi_int", std::string("int")},
                                          {"casadi_real", std::string("double")},
                                          {"mex", false},
                                          {"force_canonical", false}});
  for (const auto &f : make_codegen_functions(p)) gen.add(f);
  gen.generate(dir + "/");
}

}  // namespace balancio_nmpc
