"""Write first moves of the Python NMPC for test_nmpc.cpp.

usage: make_nmpc_reference.py <balancio.yaml> <out.csv>

Each row: x0 (6), u_prev (2), v, w, then the first move u0 (2). The
reference is the one mpc_node.py builds: x and psi integrated from (v, w)
over the horizon, dx = v, dpsi = w, the rest 0.
"""

import sys

import casadi as ca
import numpy as np
from balancio_control.model import TwsbrParams
from balancio_control.nmpc import Nmpc, NmpcConfig

from balancio_control import nmpc as nmpc_py

# Solve to a much tighter tolerance than the controller uses, so the test
# compares the optimum itself, not where each solver happened to stop.
_nlpsol = ca.nlpsol


def _tight_nlpsol(name, solver, nlp, opts):
    opts = {**opts, "ipopt.tol": 1e-10, "ipopt.acceptable_tol": 1e-10}
    return _nlpsol(name, solver, nlp, opts)


nmpc_py.ca.nlpsol = _tight_nlpsol

CASES = 40

params = TwsbrParams.from_yaml(sys.argv[1])
cfg = NmpcConfig(solver="ipopt", max_iter=500)
mpc = Nmpc(params, cfg)
N = cfg.horizon
k = np.arange(N + 1) * cfg.dt
rng = np.random.default_rng(0)
rows = []
for _ in range(CASES):
    x0 = rng.uniform(-1, 1, 6) * [0.2, 0.3, 0.3, 0.3, 1.0, 1.0]
    u_prev = rng.uniform(-0.5, 0.5, 2)
    v, w = rng.uniform(-1, 1, 2) * [0.3, 0.5]
    ref = np.zeros((6, N + 1))
    ref[0], ref[2], ref[3], ref[5] = v * k, x0[2] + w * k, v, w
    mpc.reset()
    sol = mpc.solve(x0, ref, u_prev)
    assert sol.success, sol.status
    rows.append(np.concatenate([x0, u_prev, [v, w], sol.u]))
np.savetxt(sys.argv[2], np.array(rows), fmt="%.17g", delimiter=",")
