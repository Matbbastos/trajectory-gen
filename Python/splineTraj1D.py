#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import BPoly
from math import ceil


def compute_ref_time(ref_points, max_speed):
    ref_time = 0
    for i in range(len(ref_points) - 1):
        ref_time = ref_time + abs((ref_points[i] - ref_points[i+1]/max_speed))
    return ref_time


def append_derivatives(ref_points, order=1, depth=1):
    """Appends zeros to the start and end of the ref_points array.
    The new array has the same length as the original one, but the
    depth parameter indicates how many entries of it were reassigned as
    arrays, instead of numbers, counting from the start until depth,
    and from end-depth until the end. The arrays that are assigned at
    those positions have the original entry on index 0, and the order
    param indicates how many zeros are appended to it.
    General rule to create the output array is:
        if not (start+depth < index < end-depth)
            y[i][j] = [xi, 0, 0, ... 0]  # as many zeros as 'order'
        else
            y[i][j] = xi
    """
    return [[ref_points[j] if (i == 0) else 0 for i in range(order+1)] if ((j - depth + 1 <= 0) or (j + depth - 1 >= len(ref_points)-1)) else ref_points[j] for j in range(len(ref_points))]


MAX_LIN_SPEED = 7       # [m/s]
MAX_LIN_ACCEL = 10      # [m/s^2]
MAX_LIN_JERK = 20       # [m/s^3]
SAMPLE_FREQ = 100       # [Hz]

if __name__ == "__main__":
    ref_points = [0, 2, 8, 15, 10, 22, -1, -5, 0]
    ref_time = compute_ref_time(ref_points, MAX_LIN_SPEED)
    n_points = int(ceil(SAMPLE_FREQ*ref_time))

    ref_complete = append_derivatives(ref_points, 2, 2)
    print(ref_complete)

    time_vector = np.linspace(0, ref_time, n_points)
    time_points = np.linspace(0, ref_time, len(ref_points))
    # spline = BPoly.from_derivatives(BPoly, time_points, ref_complete, 6)


# Sample code
# x = np.arange(0, 2*np.pi+np.pi/4, 2*np.pi/8)
# y = np.sin(x)
# tck = interpolate.splrep(x, y, s=0)
# xnew = np.arange(0, 2*np.pi, np.pi/50)
# ynew = interpolate.splev(xnew, tck, der=0)

# plt.figure(num=1)
# plt.plot(x, y, 'x', xnew, ynew, xnew, np.sin(xnew), x, y, 'b')
# plt.legend(['Linear', 'Cubic Spline', 'True'])
# plt.axis([-0.05, 6.33, -1.05, 1.05])
# plt.title('Cubic-spline interpolation')

# yder = interpolate.splev(xnew, tck, der=1)
# plt.figure(num=2)
# plt.plot(xnew, yder, xnew, np.cos(xnew), '--')
# plt.legend(['Cubic Spline', 'True'])
# plt.axis([-0.05, 6.33, -1.05, 1.05])
# plt.title('Derivative estimation from spline')
# plt.show()
