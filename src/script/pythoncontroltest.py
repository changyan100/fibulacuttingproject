#!/usr/bin/env python3
# secord.py - demonstrate some standard MATLAB commands
# RMM, 25 May 09

import os
import matplotlib.pyplot as plt   # MATLAB plotting functions
from control.matlab import *  # MATLAB-like functions
import numpy as np
import control

# Parameters defining the system
m = 1545.0           # system mass
k = 0.0            # spring constant
c = 3445.0            # damping constant

# System matrices
A = [[0, 1.], [-k/m, -c/m]]
B = [[0], [1/m]]
C = [[1., 0]]
sys = ss(A, B, C, 0)

# T1 = np.arange(0,  0.002, 0.001)
# T2 = T1.flatten()
# U = 100*np.sin(T2)

T, yout = control.impulse_response(sys, T=20, X0=0.0, input=20, output=None, T_num=None, transpose=False, return_x=False, squeeze=True)

print("yout, ",yout)
print("yout.t ", yout.T)
# Step response for the system
plt.figure(1)
# yout, T = step(sys)
plt.plot(T.T, yout.T)
plt.show(block=False)

# # Bode plot for the system
# plt.figure(2)
# mag, phase, om = bode(sys, logspace(-2, 2), Plot=True)
# plt.show(block=False)

# # Nyquist plot for the system
# plt.figure(3)
# nyquist(sys, logspace(-2, 2))
# plt.show(block=False)

# # Root lcous plot for the system
# rlocus(sys)

if 'PYCONTROL_TEST_EXAMPLES' not in os.environ:
    plt.show()
