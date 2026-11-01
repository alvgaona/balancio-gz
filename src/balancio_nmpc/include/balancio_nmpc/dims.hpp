// Problem sizes, with no dependencies: shared by the host code (CasADi) and
// the run-time code (acados, the EKF) that also builds for the ESP32.
#pragma once

namespace balancio_nmpc {

constexpr int NX = 6;        // state [x, theta, psi, dx, dtheta, dpsi]
constexpr int NU = 2;        // input [duty_L, duty_R]
constexpr int NZ = NX + NU;  // NMPC state: x and the previous duty
constexpr int NP = 1;        // acados stage parameter: step length h

}  // namespace balancio_nmpc
