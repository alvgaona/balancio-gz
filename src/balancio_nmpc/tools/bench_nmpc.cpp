// Solve-time benchmark of the NMPC, SQP vs SQP-RTI over horizons, in closed
// loop on a mismatched plant (no ROS, no Gazebo).
//
// usage: bench_nmpc <balancio.yaml> [runs] [initial pitch, rad]
//
// Scenario (as test_nmpc's closed loop): plant parameters off by +-10 %,
// start leaning (0.3 rad by default), then drive at 0.3 m/s and turn at 0.5 rad/s from
// 3 s to 8 s. Only the solves are timed. Recovery: the first 3 s.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "balancio_nmpc/model.hpp"
#include "balancio_nmpc/nmpc.hpp"

using namespace balancio_nmpc;
using State = std::array<double, NX>;
using Input = std::array<double, NU>;

namespace {

Params mismatched(Params p) {
  double *f[] = {&p.d, &p.l, &p.r, &p.mb, &p.mw, &p.J, &p.K, &p.I1, &p.I2, &p.I3,
                 &p.calpha, &p.max_torque, &p.motor_no_load_speed, &p.motor_inertia};
  for (int i = 0; i < 14; ++i) *f[i] *= (i % 2) ? 1.1 : 0.9;
  return p;
}

State plant_step(const casadi::Function &F, const State &x, const Input &u) {
  const auto r = F(std::vector<casadi::DM>{casadi::DM(std::vector<double>(x.begin(), x.end())),
                                           casadi::DM(std::vector<double>(u.begin(), u.end()))});
  const auto v = r[0].nonzeros();
  State xn;
  std::copy(v.begin(), v.end(), xn.begin());
  return xn;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <balancio.yaml> [runs] [initial pitch]\n", argv[0]);
    return 2;
  }
  const Params P = Params::from_yaml(argv[1]);
  const int runs = argc > 2 ? std::atoi(argv[2]) : 5;
  const double pitch0 = argc > 3 ? std::atof(argv[3]) : 0.3;

  std::printf("| solver | N | mean [ms] | p99 [ms] | max [ms] | iters | RMS pitch [deg] |"
              " speed err [m/s] | recovery RMS pitch [deg] | recovery max speed [m/s] |\n"
              "|---|---|---|---|---|---|---|---|---|---|---|\n");
  for (bool rti : {false, true}) {
    for (int N : {25, 15, 10}) {
      NmpcConfig cfg;
      cfg.rti = rti;
      cfg.horizon = N;
      const auto plant = make_rk4_function(mismatched(P), cfg.dt, 20);
      std::vector<double> ms;
      double iters = 0, pitch2 = 0, verr2 = 0, rec2 = 0, rec_max = 0, sat = 0;
      int n_late = 0, n_rec = 0, failures = 0;
      for (int run = 0; run < runs; ++run) {
        Nmpc mpc(P, cfg);
        State x{0.0, pitch0, 0.0, 0.0, 0.0, 0.0};
        Input u{0.0, 0.0};
        double x_ref = 0.0, psi_ref = 0.0;
        for (int i = 0; i < static_cast<int>(8.0 / cfg.dt); ++i) {
          const double t = i * cfg.dt;
          const double v = t > 3.0 ? 0.3 : 0.0, w = t > 3.0 ? 0.5 : 0.0;
          std::vector<State> ref(N + 1);
          for (int k = 0; k <= N; ++k)
            ref[k] = {x_ref + v * k * cfg.dt, 0.0, psi_ref + w * k * cfg.dt, v, 0.0, w};
          const auto t0 = std::chrono::steady_clock::now();
          const auto sol = mpc.solve(x, ref, u);
          ms.push_back(
              std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0)
                  .count());
          iters += sol.iterations;
          failures += !sol.success();
          u = sol.u;
          sat += std::fabs(u[0]) > cfg.max_duty - 1e-3;
          x = plant_step(plant, x, u);
          x_ref += v * cfg.dt;
          psi_ref += w * cfg.dt;
          if (t < 3.0) {  // recovering from the initial lean
            rec2 += x[1] * x[1];
            rec_max = std::max(rec_max, std::fabs(x[3]));
            ++n_rec;
          }
          if (t > 4.0) {  // settled on the drive command
            pitch2 += x[1] * x[1];
            verr2 += (x[3] - v) * (x[3] - v);
            ++n_late;
          }
        }
      }
      std::sort(ms.begin(), ms.end());
      double mean = 0;
      for (double m : ms) mean += m;
      mean /= ms.size();
      std::printf("| %s | %d | %.3f | %.3f | %.3f | %.2f | %.2f | %.4f | %.2f | %.3f | %.1f %% |%s\n",
                  rti ? "RTI" : "SQP", N, mean, ms[ms.size() * 99 / 100], ms.back(),
                  iters / ms.size(), std::sqrt(pitch2 / n_late) * 180 / M_PI,
                  std::sqrt(verr2 / n_late), std::sqrt(rec2 / n_rec) * 180 / M_PI, rec_max, 100.0 * sat / ms.size(), failures ? " solver failures!" : "");
    }
  }
  return 0;
}
