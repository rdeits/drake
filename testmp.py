import sys
sys.path.append("build/install/lib/python2.7/dist-packages")

import numpy as np

import pydrake.mathematicalprogram as mp
prog = mp.MathematicalProgram()
x = prog.NewContinuousVariables(3)

A = np.array([[1.0, 2, 3], [0, 1, -1], [0, 0, 0]])
b_lb = np.array([0, -np.inf, -1])
b_ub = np.array([10, 3, 0])

prog.AddLinearConstraint(A, b_lb, b_ub)
