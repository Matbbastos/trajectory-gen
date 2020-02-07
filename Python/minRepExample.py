#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import BPoly, splrep, splev, Akima1DInterpolator


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


if __name__ == "__main__":
    ref_points = [0, 2, 1, 4, 2, 0]
    time_vector = np.linspace(0, 10, 100)
    time_points = np.linspace(0, 10, 6)

    ref_complete = append_derivatives(ref_points, 1, 2)
    print("Reference points with derivatives: ", ref_complete)

    ref_spline = BPoly.from_derivatives(time_points, ref_complete)
    spline = ref_spline(time_vector)

    # splineRep = splrep(time_points, ref_points, k=3)
    # spline = splev(time_vector, splineRep)

    # akima = Akima1DInterpolator(time_points, ref_points)
    # akimaSpl = akima(time_vector)

    # Plots
    plt.figure('Python Plot')
    plt.plot(time_points, ref_points, '--^r', time_vector, spline, '-b')
    plt.legend(['Linear', 'Spline'])
    plt.title('Spline interpolation')
    plt.show()
