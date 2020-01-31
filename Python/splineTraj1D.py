#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import BPoly, splrep, splev
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
    return [[ref_points[j] if (i == 0) else 0 for i in range(order+1)] if ((j - depth + 1 <= 0) or (j + depth - 1 >= len(ref_points)-1)) else [ref_points[j]] for j in range(len(ref_points))]


MAX_LIN_SPEED = 7       # [m/s]
MAX_LIN_ACCEL = 10      # [m/s^2]
MAX_LIN_JERK = 20       # [m/s^3]
SAMPLE_FREQ = 100       # [Hz]

if __name__ == "__main__":
    ref_points = [0, 2, 8, 15, 10, 22, -1, -5, 0]
    ref_time = compute_ref_time(ref_points, MAX_LIN_SPEED)
    n_points = int(ceil(SAMPLE_FREQ*ref_time))

    ref_complete = append_derivatives(ref_points, 1, 2)
    print("Reference points with derivatives: ", ref_complete)

    time_vector = np.linspace(0, ref_time, num=n_points)
    time_points = np.linspace(0, ref_time, num=len(ref_points))
    print("Time points: ", time_points)
    print("Time vector: ", time_vector)
    # bSpline = BPoly.from_derivatives(time_points, ref_complete, 3)
    splineRep = splrep(time_points, ref_points, k=3)
    spline = splev(time_vector, splineRep)
    print(spline)

    # Plots
    plt.figure('Python Plot')
    plt.plot(time_points, ref_points, '--*r', time_vector, spline, '-b')
    plt.legend(['Linear', 'Spline'])
    plt.title('Spline interpolation')
    plt.show()
