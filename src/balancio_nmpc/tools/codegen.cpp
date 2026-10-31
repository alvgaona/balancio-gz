// Build-time tool: write the model functions as C for acados and the EKF.
// usage: balancio_codegen <balancio.yaml> <output dir>
#include <cstdio>

#include "balancio_nmpc/codegen.hpp"

int main(int argc, char **argv) {
  if (argc != 3) {
    std::fprintf(stderr, "usage: %s <balancio.yaml> <output dir>\n", argv[0]);
    return 2;
  }
  const auto p = balancio_nmpc::Params::from_yaml(argv[1]);
  balancio_nmpc::generate_c_code(p, argv[2], "balancio_model");
  return 0;
}
