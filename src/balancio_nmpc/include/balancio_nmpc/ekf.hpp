// State estimation from the IMU alone, the C++ counterpart of
// balancio_control/estimator.py (ImuEkf).
//
// An extended Kalman filter runs the NMPC's model (robot + DC motors),
// driven by the duties sent, and corrects it with the IMU: pitch and yaw
// from the orientation, their rates from the gyro. dx shows up through the
// motors' back-EMF; x is its integral and drifts. Inputs and IMU samples are
// timestamped, so an irregular control loop doesn't turn into a speed error.
//
// No CasADi at run time: the model step and its Jacobian are the generated
// C function balancio_ekf_step, and the matrix math is fixed-size, so this
// runs on a microcontroller too.
#pragma once

#include <array>
#include <deque>
#include <optional>
#include <vector>

#include "balancio_nmpc/model.hpp"

namespace balancio_nmpc {

using State = std::array<double, NX>;
using Input = std::array<double, NU>;

double wrap_angle(double a);
// ZYX Euler angles (roll, pitch, yaw) of the quaternion (x, y, z, w).
std::array<double, 3> quat_to_euler(double qx, double qy, double qz, double qw);

class ImuEkf {
 public:
  static constexpr double MAX_STEP = 0.005;  // longest model step [s]

  // dt: nominal control period. q: process noise std per dt.
  // r: IMU noise std [theta, psi, dtheta, dpsi].
  ImuEkf(double dt, const std::array<double, NX> &q = {1e-4, 1e-3, 1e-3, 1e-3, 5e-2, 2e-2},
         const std::array<double, 4> &r = {3e-3, 1e-2, 5e-3, 5e-3});

  void update_imu(const std::array<double, 4> &quat, const std::array<double, 3> &gyro,
                  double stamp);
  // Record a duty sent to the motors at time stamp.
  void set_input(double stamp, const Input &u);
  // Restart from the latest IMU sample with the robot at rest.
  void reset();
  // Fuse the latest IMU sample and return the state predicted at now.
  State step(double now);

  bool ready() const { return have_imu_; }
  double roll() const { return roll_; }
  double X() const { return X_; }  // dead-reckoned planar position
  double Y() const { return Y_; }

 private:
  using Mat = std::array<double, NX * NX>;  // row-major

  void model_step(State &x, const Input &u, double h, Mat *A);
  void propagate(State &x, Mat *P, double t0, double t1);

  double dt_;
  Mat Q_{};
  std::array<double, 4> R_{};

  bool have_imu_ = false;
  double roll_ = 0.0;
  std::array<double, 4> z_{};  // theta, psi, dtheta, dpsi
  double z_time_ = 0.0;
  double psi_ = 0.0;
  std::optional<double> yaw_last_;

  std::optional<State> x_;  // filter state at time t_
  double t_ = 0.0;
  Mat P_{};
  std::deque<std::pair<double, Input>> inputs_;  // each applied until the next
  std::optional<double> t_out_;
  double X_ = 0.0, Y_ = 0.0;

  // Work memory for the generated function.
  std::vector<const double *> arg_;
  std::vector<double *> res_;
  std::vector<int> iw_;
  std::vector<double> w_;
};

}  // namespace balancio_nmpc
