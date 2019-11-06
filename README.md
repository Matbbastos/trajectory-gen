# Trajectory Generation
Matlab (and eventually Python) scripts used to generate spline interpolated trajectories using reference points.
The scripts interpolate using k-th order splines representing the position references, and verify constraints on the
1st, 2nd and 2rd derivatives (namely speed, acceleration and jerk). If the derivatives are not properly bounded by the
desired limits, the time in which the position splines were first computed is stretched, and a new verification takes
place. The process iterates until all the conditions are met. Also, it is possible to define start and end conditions
for the derivatives of the position spline curve.
There are 3 versions of the script, to be used in correspondence with the desired number of dimensions of the movement
(line, plane, space).  
