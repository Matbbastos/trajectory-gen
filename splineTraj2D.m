addpath('Stretch');

%% Initial settings
refPointsX = [0 1 5 10 20 12  2 -5 0];     % [m] position in X-axis
refPointsY = [0 0 0 -1 -5  2 10 10 0];     % [m] position in Y-axis

lenRef = max(length(refPointsX), length(refPointsY));

maxLinSpeed = 7;    % [m/s]
maxLinAccel = 10;   % [m/s^2]
maxLinJerk = 20;    % [m/s^3]
sampleFreq = 100;   % [Hz]

refTime = 0;    % [s] sum of time to reach each point in a straight line from the previous point

for i = 1:(lenRef - 1)
    refTime = refTime + norm([refPointsX(i+1) refPointsY(i+1)] - [refPointsX(i) refPointsY(i)])/maxLinSpeed;
end

%% High order spline with derivatives and stretching
order = 6;
time = linspace(0, refTime, sampleFreq*refTime);
splTimeRef = linspace(0, refTime, lenRef);

% Add conditions to derivatives (speed = acceleration = 0 at start and end)
refPositionWderX = addZeros(refPointsX, [1, 1, lenRef, lenRef]);
refPositionWderY = addZeros(refPointsY, [1, 1, lenRef, lenRef]);

dSplTimeRef = addZeros(splTimeRef, [1, 1]);
lenTime = length(dSplTimeRef);
dSplTimeRef = addValueAt(dSplTimeRef, [lenTime, lenTime], dSplTimeRef(end));

% Create curves
knots = optknt(dSplTimeRef, order);
hSplCurveX = spapi(knots, dSplTimeRef, refPositionWderX);
hSplPointsX = fnval(time, hSplCurveX);

hSplCurveY = spapi(knots, dSplTimeRef, refPositionWderY);
hSplPointsY = fnval(time, hSplCurveY);

newSplineXY = [hSplPointsX ; hSplPointsY];

[curvesX, curvesY, linCurves] = computeDiff(time, hSplPointsX, hSplPointsY);
    
% Check constraints
failIndexSpeed = checkConstraints(linCurves(1,1:end), maxLinSpeed);
failIndexAccel = checkConstraints(linCurves(2,1:end-1), maxLinAccel);
failIndexJerk = checkConstraints(linCurves(3,1:end-2), maxLinJerk);

% Stretch curve if necessary
factor = 1.05;
doneStretching = false;
i = 0;

while ~doneStretching
    if ~isempty(failIndexSpeed) || ~isempty(failIndexAccel) || ~isempty(failIndexJerk)
        [newTime, newSplineXY] = stretch2DCurve(refTime, factor^i, lenRef, [refPositionWderX ; refPositionWderY], sampleFreq, order);

        [curvesX, curvesY, linCurves] = computeDiff(newTime, newSplineXY(1,1:end), newSplineXY(2,1:end));

        failIndexSpeed = checkConstraints(linCurves(1,1:end), maxLinSpeed);
        failIndexAccel = checkConstraints(linCurves(2,1:end-1), maxLinAccel);
        failIndexJerk = checkConstraints(linCurves(3,1:end-2), maxLinJerk);

        i = i+1;
        continue
    end
    doneStretching = true;
    fprintf('\nStretched time %d times, by a total factor of %f\n', i, factor^i)
    fprintf('Original duration of trajectory: %f s\nNew duration: %f s\n', refTime, newTime(end))
end

%% Plots
% X-axis
f = figure('NumberTitle', 'off', 'Name', 'High order 2D Fitting Test - X-axis');
f.WindowState = 'maximized';

subplot(4, 1, 1)
hold on
grid on
plot(newTime, newSplineXY(1,1:end), '-r', 'LineWidth', 1.5)
plot(linspace(0, newTime(end), lenRef), refPointsX, '-r^', 'LineWidth', 1.5, 'LineStyle', 'none')
legend({'Position'; 'Ref points'})
ylabel('Distance [m]')
title('X-Axis')

subplot(4, 1, 2)
hold on
grid on
plot(newTime(1:end-1), curvesX(1,1:end), '-b', 'LineWidth', 1.5)
legend({'Speed'})
ylabel('Speed [m/s]')

subplot(4, 1, 3)
hold on
grid on
plot(newTime(1:end-2), curvesX(2,1:end-1), '-m', 'LineWidth', 1.5)
legend({'Acceleration'})
ylabel('Acceleration [m/s^2]')

subplot(4, 1, 4)
hold on
grid on
plot(newTime(1:end-3), curvesX(3,1:end-2), '-g', 'LineWidth', 1.5)
legend({'Jerk'})
xlabel('Time [s]')
ylabel('Jerk [m/s^3]')

% Y-axis
f = figure('NumberTitle', 'off', 'Name', 'High order 2D Fitting - Y-axis');
f.WindowState = 'maximized';

subplot(4, 1, 1)
hold on
grid on
plot(newTime, newSplineXY(2,1:end), '-r', 'LineWidth', 1.5)
plot(linspace(0, newTime(end), lenRef), refPointsY, '-r^', 'LineWidth', 1.5, 'LineStyle', 'none')
legend({'Position'; 'Ref points'})
ylabel('Distance [m]')
title('X-Axis')

subplot(4, 1, 2)
hold on
grid on
plot(newTime(1:end-1), curvesY(1,1:end), '-b', 'LineWidth', 1.5)
legend({'Speed'})
ylabel('Speed [m/s]')

subplot(4, 1, 3)
hold on
grid on
plot(newTime(1:end-2), curvesY(2,1:end-1), '-m', 'LineWidth', 1.5)
legend({'Acceleration'})
ylabel('Acceleration [m/s^2]')

subplot(4, 1, 4)
hold on
grid on
plot(newTime(1:end-3), curvesY(3,1:end-2), '-g', 'LineWidth', 1.5)
legend({'Jerk'})
xlabel('Time [s]')
ylabel('Jerk [m/s^3]')

% Linear
f = figure('NumberTitle', 'off', 'Name', 'High order 2D Fitting - XY-Plane');
f.WindowState = 'maximized';

hold on
grid on
plot(newSplineXY(1,1:end), newSplineXY(2,1:end), '-r', 'LineWidth', 1.5)
plot(refPointsX, refPointsY, '-r^', 'LineWidth', 1.5, 'LineStyle', 'none')
legend({'Position'; 'Ref points'})
xlabel('X-axis [m]')
ylabel('Y-axis [m]')
axis equal
title('Position in XY-Plane')


f = figure('NumberTitle', 'off', 'Name', 'High order 2D Fitting - Linear Derivatives');
f.WindowState = 'maximized';

subplot(3, 1, 1)
hold on
grid on
plot(newTime(1:end-1), linCurves(1,1:end), '-b', 'LineWidth', 1.5)
plot(newTime, maxLinSpeed*ones(1, length(newTime)), '--k')
plot(newTime, -maxLinSpeed*ones(1, length(newTime)), '--k')
legend({'Speed', 'Limits'})
ylabel('Speed [m/s]')
title('Linear Derivatives')

subplot(3, 1, 2)
hold on
grid on
plot(newTime(1:end-2), linCurves(2,1:end-1), '-m', 'LineWidth', 1.5)
plot(newTime, maxLinAccel*ones(1, length(newTime)), '--k')
plot(newTime, -maxLinAccel*ones(1, length(newTime)), '--k')
legend({'Acceleration', 'Limits'})
ylabel('Acceleration [m/s^2]')

subplot(3, 1, 3)
hold on
grid on
plot(newTime(1:end-3), linCurves(3,1:end-2), '-g', 'LineWidth', 1.5)
plot(newTime, maxLinJerk*ones(1, length(newTime)), '--k')
plot(newTime, -maxLinJerk*ones(1, length(newTime)), '--k')
legend({'Jerk', 'Limits'})
xlabel('Time [s]')
ylabel('Jerk [m/s^3]')
