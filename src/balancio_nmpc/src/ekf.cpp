#include "balancio_nmpc/ekf.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

extern "C" {
#include "balancio_model.h"
}

namespace balancio_nmpc {

namespace {

constexpr int NZ_MEAS = 4;
// Measured state components: theta, psi, dtheta, dpsi.
constexpr int MEAS[NZ_MEAS] = {1, 2, 4, 5};

using Mat = std::array<double, NX * NX>;

Mat matmul(const Mat &a, const Mat &b) {
  Mat c{};
  for (int i = 0; i < NX; ++i)
    for (int k = 0; k < NX; ++k)
      for (int j = 0; j < NX; ++j) c[i * NX + j] += a[i * NX + k] * b[k * NX + j];
  return c;
}

Mat transpose(const Mat &a) {
  Mat t{};
  for (int i = 0; i < NX; ++i)
    for (int j = 0; j < NX; ++j) t[j * NX + i] = a[i * NX + j];
  return t;
}

// Solve S X = B for X (S: n x n, B: n x m, both row-major, n small), by
// Gaussian elimination with partial pivoting. B is overwritten with X.
void solve(std::array<double, NZ_MEAS * NZ_MEAS> S, std::array<double, NZ_MEAS * NX> &B) {
  constexpr int n = NZ_MEAS, m = NX;
  for (int c = 0; c < n; ++c) {
    int piv = c;
    for (int r = c + 1; r < n; ++r)
      if (std::fabs(S[r * n + c]) > std::fabs(S[piv * n + c])) piv = r;
    if (piv != c) {
      for (int j = 0; j < n; ++j) std::swap(S[c * n + j], S[piv * n + j]);
      for (int j = 0; j < m; ++j) std::swap(B[c * m + j], B[piv * m + j]);
    }
    for (int r = c + 1; r < n; ++r) {
      const double f = S[r * n + c] / S[c * n + c];
      for (int j = c; j < n; ++j) S[r * n + j] -= f * S[c * n + j];
      for (int j = 0; j < m; ++j) B[r * m + j] -= f * B[c * m + j];
    }
  }
  for (int c = n - 1; c >= 0; --c) {
    for (int j = 0; j < m; ++j) {
      double v = B[c * m + j];
      for (int k = c + 1; k < n; ++k) v -= S[c * n + k] * B[k * m + j];
      B[c * m + j] = v / S[c * n + c];
    }
  }
}

}  // namespace

double wrap_angle(double a) {
  // Python's % is a floored modulo; std::fmod truncates, so fix the sign.
  double r = std::fmod(a + M_PI, 2.0 * M_PI);
  if (r < 0) r += 2.0 * M_PI;
  return r - M_PI;
}

std::array<double, 3> quat_to_euler(double qx, double qy, double qz, double qw) {
  const double roll = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
  const double pitch = std::asin(std::clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0));
  const double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
  return {roll, pitch, yaw};
}

ImuEkf::ImuEkf(double dt, const std::array<double, NX> &q, const std::array<double, 4> &r)
    : dt_(dt) {
  for (int i = 0; i < NX; ++i) Q_[i * NX + i] = q[i] * q[i];
  for (int i = 0; i < NZ_MEAS; ++i) R_[i] = r[i] * r[i];
  int sz_arg, sz_res, sz_iw, sz_w;
  balancio_ekf_step_work(&sz_arg, &sz_res, &sz_iw, &sz_w);
  arg_.assign(sz_arg, nullptr);
  res_.assign(sz_res, nullptr);
  iw_.resize(sz_iw);
  w_.resize(sz_w);
  // The step's outputs must be dense for the copies in model_step.
  for (int i = 0; i < 2; ++i)
    if (balancio_ekf_step_sparsity_out(i)[2] != 1)
      throw std::logic_error("balancio_ekf_step: expected dense outputs");
}

void ImuEkf::update_imu(const std::array<double, 4> &quat, const std::array<double, 3> &gyro,
                        double stamp) {
  const auto [roll, pitch, yaw] = quat_to_euler(quat[0], quat[1], quat[2], quat[3]);
  if (!yaw_last_)
    psi_ = yaw;
  else
    psi_ += wrap_angle(yaw - *yaw_last_);
  yaw_last_ = yaw;
  const double q = gyro[1], r = gyro[2];
  const double dtheta = q * std::cos(roll) - r * std::sin(roll);
  const double dpsi = (q * std::sin(roll) + r * std::cos(roll)) / std::max(std::cos(pitch), 1e-3);
  roll_ = roll;
  z_ = {pitch, psi_, dtheta, dpsi};
  z_time_ = stamp;
  have_imu_ = true;
}

void ImuEkf::set_input(double stamp, const Input &u) { inputs_.emplace_back(stamp, u); }

void ImuEkf::reset() {
  const double x0 = x_ ? (*x_)[0] : 0.0;
  x_ = State{x0, z_[0], z_[1], 0.0, z_[2], z_[3]};
  t_ = z_time_;
  P_ = {};
  const double p0[NX] = {0.0, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2};
  for (int i = 0; i < NX; ++i) P_[i * NX + i] = p0[i];
}

void ImuEkf::model_step(State &x, const Input &u, double h, Mat *A) {
  State xn;
  Mat Acol;  // the generated Jacobian is column-major
  arg_[0] = x.data();
  arg_[1] = u.data();
  arg_[2] = &h;
  res_[0] = xn.data();
  res_[1] = A ? Acol.data() : nullptr;
  balancio_ekf_step(arg_.data(), res_.data(), iw_.data(), w_.data(), 0);
  x = xn;
  if (A) *A = transpose(Acol);
}

void ImuEkf::propagate(State &x, Mat *P, double t0, double t1) {
  // Pieces of constant duty covering [t0, t1], each split into steps <= MAX_STEP.
  Input u{};
  for (const auto &[t, ui] : inputs_)
    if (t <= t0) u = ui;
  std::vector<double> times;
  for (const auto &[t, ui] : inputs_)
    if (t0 < t && t < t1) times.push_back(t);
  times.push_back(t1);

  double a = t0;
  for (double b : times) {
    const int n = std::max(1, static_cast<int>(std::ceil((b - a) / MAX_STEP - 1e-9)));
    const double h = (b - a) / n;
    for (int i = 0; i < n; ++i) {
      if (P) {
        Mat A;
        model_step(x, u, h, &A);
        Mat Pn = matmul(matmul(A, *P), transpose(A));
        for (int j = 0; j < NX * NX; ++j) Pn[j] += Q_[j] * (h / dt_);
        *P = Pn;
      } else {
        model_step(x, u, h, nullptr);
      }
    }
    a = b;
    for (const auto &[t, ui] : inputs_)
      if (t == b) {
        u = ui;
        break;
      }
  }
}

State ImuEkf::step(double now) {
  if (!x_) {
    reset();
  } else if (z_time_ > t_) {
    propagate(*x_, &P_, t_, z_time_);
    t_ = z_time_;

    // S = H P H' + R, K = P H' S^-1 (H selects the measured components).
    std::array<double, NZ_MEAS * NZ_MEAS> S{};
    std::array<double, NZ_MEAS * NX> HP{};  // becomes K'
    for (int i = 0; i < NZ_MEAS; ++i) {
      for (int j = 0; j < NZ_MEAS; ++j) S[i * NZ_MEAS + j] = P_[MEAS[i] * NX + MEAS[j]];
      S[i * NZ_MEAS + i] += R_[i];
      for (int j = 0; j < NX; ++j) HP[i * NX + j] = P_[MEAS[i] * NX + j];
    }
    solve(S, HP);  // S is symmetric, so S^-1 H P = K'
    State &x = *x_;
    double innov[NZ_MEAS];
    for (int i = 0; i < NZ_MEAS; ++i) innov[i] = z_[i] - x[MEAS[i]];
    for (int j = 0; j < NX; ++j)
      for (int i = 0; i < NZ_MEAS; ++i) x[j] += HP[i * NX + j] * innov[i];

    // Joseph form: P = (I - K H) P (I - K H)' + K R K'
    Mat IKH{};
    for (int i = 0; i < NX; ++i) IKH[i * NX + i] = 1.0;
    for (int i = 0; i < NX; ++i)
      for (int m = 0; m < NZ_MEAS; ++m) IKH[i * NX + MEAS[m]] -= HP[m * NX + i];
    Mat Pn = matmul(matmul(IKH, P_), transpose(IKH));
    for (int i = 0; i < NX; ++i)
      for (int j = 0; j < NX; ++j)
        for (int m = 0; m < NZ_MEAS; ++m) Pn[i * NX + j] += HP[m * NX + i] * R_[m] * HP[m * NX + j];
    P_ = Pn;
  }
  // Forget duties that ended before t_ (keep the one in force).
  while (inputs_.size() > 1 && inputs_[1].first <= t_) inputs_.pop_front();

  State x_now = *x_;
  propagate(x_now, nullptr, t_, std::max(now, t_));

  // Planar dead reckoning
  if (t_out_) {
    const double ds = x_now[3] * (now - *t_out_);
    X_ += ds * std::cos(x_now[2]);
    Y_ += ds * std::sin(x_now[2]);
  }
  t_out_ = now;
  return x_now;
}

}  // namespace balancio_nmpc
