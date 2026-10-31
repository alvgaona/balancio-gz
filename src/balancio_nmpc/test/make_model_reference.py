"""Write reference values of the Python model for test_model.cpp.

usage: make_model_reference.py <balancio.yaml> <out.csv>

Each row: state (6), duty (2), f(state, duty) (6), RK4 step over DT (6).
"""

import sys

import numpy as np
from balancio_control.model import (
    TwsbrParams,
    make_dynamics_function,
    make_rk4_function,
)

DT = 0.02
POINTS = 300

params = TwsbrParams.from_yaml(sys.argv[1])
f = make_dynamics_function(params)
F = make_rk4_function(params, DT)
rng = np.random.default_rng(0)
lo = np.array([-2.0, -0.6, -3.0, -1.0, -5.0, -3.0, -1.0, -1.0])
hi = -lo
rows = []
for _ in range(POINTS):
    xu = rng.uniform(lo, hi)
    x, u = xu[:6], xu[6:]
    rows.append(np.concatenate([x, u, np.ravel(f(x, u)), np.ravel(F(x, u))]))
np.savetxt(sys.argv[2], np.array(rows), fmt="%.17g", delimiter=",")
