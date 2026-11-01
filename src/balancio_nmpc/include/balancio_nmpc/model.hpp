// Two-wheeled self-balancing robot dynamics, C++ port of
// balancio_control/model.py (itself a port of alvgaona/twsbr):
//
//   M(q) q_ddot + C(q, q_dot) q_dot + D q_dot + G(q) = B tau
//
//   q = [x, theta, psi]      forward distance, pitch, yaw
//   u = [duty_L, duty_R]     motor PWM duty in [-1, 1]
//   state = [x, theta, psi, dx, dtheta, dpsi]
//
// The wheels are driven by DC gear motors from a fixed battery voltage:
//
//   tau = max_torque * (duty - w / motor_no_load_speed)
//
// with w the wheel speed relative to the body, and the motor rotor inertia
// (reflected to the wheel, motor_inertia) spins with w.
#pragma once

#include <casadi/casadi.hpp>
#include <string>

#include "balancio_nmpc/dims.hpp"

namespace balancio_nmpc {

// Names follow balancio_description/config/balancio.yaml.
struct Params {
  double d;                    // distance between wheels [m]
  double l;                    // wheel axle -> body center of mass [m]
  double r;                    // wheel radius [m]
  double mb;                   // body mass [kg]
  double mw;                   // mass of each wheel [kg]
  double J;                    // wheel inertia about its spin axis [kg m^2]
  double K;                    // wheel inertia about the vertical axis [kg m^2]
  double I1, I2, I3;           // body inertia about x, y, z at the CoM [kg m^2]
  double calpha;               // viscous friction on the wheel axle [N m s/rad]
  double max_torque;           // stall torque at full duty [N m]
  double motor_no_load_speed;  // [rad/s]
  double motor_inertia;        // rotor inertia at the wheel [kg m^2]
  double g = 9.81;             // [m/s^2]

  static Params from_yaml(const std::string &path);
};

// Wheel speeds relative to the body (= motor shaft speeds) [rad/s].
std::pair<casadi::SX, casadi::SX> wheel_speeds(const casadi::SX &state, const Params &p);

// Wheel torques [tau_L, tau_R] for duty u (no clipping).
std::pair<casadi::SX, casadi::SX> motor_torques(const casadi::SX &state, const casadi::SX &u,
                                                const Params &p);

// State derivative for duty u.
casadi::SX continuous_dynamics(const casadi::SX &state, const casadi::SX &u, const Params &p);

// f(x, u) -> dx/dt
casadi::Function make_dynamics_function(const Params &p);

// One RK4 step of length h (a symbol or a number), zero-order-hold input.
casadi::SX rk4_step(const casadi::SX &x, const casadi::SX &u, const casadi::SX &h, const Params &p);

// Discrete-time model x_{k+1} = F(x_k, u_k), RK4 with zero-order-hold input.
casadi::Function make_rk4_function(const Params &p, double dt, int substeps = 1);

}  // namespace balancio_nmpc
