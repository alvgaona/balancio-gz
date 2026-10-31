#include "balancio_nmpc/model.hpp"

#include <yaml-cpp/yaml.h>

namespace balancio_nmpc {

using casadi::SX;

Params Params::from_yaml(const std::string &path) {
  const YAML::Node y = YAML::LoadFile(path);
  auto get = [&](const char *key) { return y[key].as<double>(); };
  Params p{};
  p.d = get("d");
  p.l = get("l");
  p.r = get("r");
  p.mb = get("mb");
  p.mw = get("mw");
  p.J = get("J");
  p.K = get("K");
  p.I1 = get("I1");
  p.I2 = get("I2");
  p.I3 = get("I3");
  p.calpha = get("calpha");
  p.max_torque = get("max_torque");
  p.motor_no_load_speed = get("motor_no_load_speed");
  p.motor_inertia = get("motor_inertia");
  p.g = y["g"] ? get("g") : 9.81;
  return p;
}

std::pair<SX, SX> wheel_speeds(const SX &state, const Params &p) {
  const SX dx = state(3), dtheta = state(4), dpsi = state(5);
  SX w_l = (dx - p.d / 2 * dpsi) / p.r - dtheta;
  SX w_r = (dx + p.d / 2 * dpsi) / p.r - dtheta;
  return {w_l, w_r};
}

std::pair<SX, SX> motor_torques(const SX &state, const SX &u, const Params &p) {
  auto [w_l, w_r] = wheel_speeds(state, p);
  const double k = p.max_torque / p.motor_no_load_speed;
  return {p.max_torque * u(0) - k * w_l, p.max_torque * u(1) - k * w_r};
}

SX continuous_dynamics(const SX &state, const SX &u, const Params &p) {
  const SX theta = state(1), dx = state(3), dtheta = state(4), dpsi = state(5);

  const SX s = sin(theta);
  const SX c = cos(theta);
  const double k = p.I3 - p.I1 - p.mb * p.l * p.l;
  const double r2 = p.r * p.r;
  const double d2 = p.d * p.d;

  // Mass matrix. The rotors add Jm (dx / r - dtheta)^2 + Jm (d dpsi / 2 r)^2
  // to the kinetic energy (two motors, relative wheel speeds).
  const double jm = p.motor_inertia;
  const double a11 = p.mb + 2 * p.mw + 2 * p.J / r2 + 2 * jm / r2;
  const SX a12 = p.mb * p.l * c - 2 * jm / p.r;
  const double a22 = p.I2 + p.mb * p.l * p.l + 2 * jm;
  const SX a33 = p.I3 + 2 * p.K + p.mw * d2 / 2 + (p.J + jm) * d2 / (2 * r2) - k * s * s;

  // Coriolis matrix
  const SX c12 = -p.mb * p.l * dtheta * s;
  const SX c13 = -p.mb * p.l * dpsi * s;
  const SX c23 = k * dpsi * s * c;
  const SX c31 = p.mb * p.l * dpsi * s;
  const SX c32 = -k * dpsi * s * c;
  const SX c33 = -k * dtheta * s * c;

  // Damping matrix
  const double d11 = 2 * p.calpha / r2;
  const double d12 = -2 * p.calpha / p.r;
  const double d22 = 2 * p.calpha;
  const double d33 = p.calpha * d2 / (2 * r2);

  auto [tau_l, tau_r] = motor_torques(state, u, p);

  // Right-hand side: B u - C q_dot - D q_dot - G
  const SX f1 = (tau_l + tau_r) / p.r - (c12 * dtheta + c13 * dpsi) - (d11 * dx + d12 * dtheta);
  const SX f2 = -(tau_l + tau_r) - c23 * dpsi - (d12 * dx + d22 * dtheta) + p.mb * p.l * p.g * s;
  const SX f3 = p.d / (2 * p.r) * (tau_r - tau_l) - (c31 * dx + c32 * dtheta + c33 * dpsi) -
                d33 * dpsi;

  // M is block diagonal: 2x2 (x, theta) and scalar (psi)
  const SX det = a11 * a22 - a12 * a12;
  const SX ddx = (a22 * f1 - a12 * f2) / det;
  const SX ddtheta = (-a12 * f1 + a11 * f2) / det;
  const SX ddpsi = f3 / a33;

  return SX::vertcat({dx, dtheta, dpsi, ddx, ddtheta, ddpsi});
}

casadi::Function make_dynamics_function(const Params &p) {
  const SX x = SX::sym("x", NX);
  const SX u = SX::sym("u", NU);
  return casadi::Function("f", {x, u}, {continuous_dynamics(x, u, p)}, {"x", "u"}, {"dx"});
}

SX rk4_step(const SX &x, const SX &u, const SX &h, const Params &p) {
  const SX k1 = continuous_dynamics(x, u, p);
  const SX k2 = continuous_dynamics(x + h / 2 * k1, u, p);
  const SX k3 = continuous_dynamics(x + h / 2 * k2, u, p);
  const SX k4 = continuous_dynamics(x + h * k3, u, p);
  return x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
}

casadi::Function make_rk4_function(const Params &p, double dt, int substeps) {
  const SX x = SX::sym("x", NX);
  const SX u = SX::sym("u", NU);
  SX xn = x;
  for (int i = 0; i < substeps; ++i) xn = rk4_step(xn, u, dt / substeps, p);
  return casadi::Function("F", {x, u}, {xn}, {"x", "u"}, {"x_next"});
}

}  // namespace balancio_nmpc
