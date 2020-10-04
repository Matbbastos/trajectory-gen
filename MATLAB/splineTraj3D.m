addpath('Stretch');

%% Initial settings
lenRef = 30;

path = linspace(0, 3*2*pi, lenRef);

refPointsX = linspace(0, 5, lenRef) + cos(path);        % [m] position in X-axis
refPointsY = linspace(0, -3, lenRef) + sin(path);        % [m] position in Y-axis
refPointsZ = [linspace(0, 10, lenRef/3) 10*ones(1, lenRef/3) linspace(10, 5, lenRef/6) linspace(5, 10, lenRef/6)];        % [m] position in Z-axis

distanceRef = zeros(size(refPointsX));
for i = 1:(lenRef)
    distanceRef(i) = norm([refPointsX(i) refPointsY(i) refPointsZ(i)]);
end

maxLinSpeed = 7;    % [m/s]
maxLinAccel = 10;   % [m/s^2]
maxLinJerk = 20;    % [m/s^3]
sampleFreq = 100;   % [Hz]

refTime = 0;    % [s] sum of time to reach each point in a straight line from the previous point

for i = 1:(lenRef - 1)
    refTime = refTime + norm([refPointsX(i+1) refPointsY(i+1) refPointsZ(i+1)] - [refPointsX(i) refPointsY(i) refPointsZ(i)])/maxLinSpeed;
end

%% High order spline with derivatives and stretching
order = 6;
time = linspace(0, refTime, sampleFreq*refTime);
splTimeRef = linspace(0, refTime, lenRef);

% Reference scenario
refDistanceX = interp1(splTimeRef, refPointsX, time);
refDistanceY = interp1(splTimeRef, refPointsY, time);
refDistanceZ = interp1(splTimeRef, refPointsZ, time);

linDistanceRef = sqrt(sum([refDistanceX; refDistanceY; refDistanceZ;].^2));
linSpeedRef = diff(linDistanceRef)./diff(time);
linAccelRef = diff(linSpeedRef)./diff(time(1:end-1));

% Add conditions to derivatives (speed = acceleration = 0 at start and end)
zeroJerk = false;   % if zero jerk at start and end is desired, this should be 'true'

if ~zeroJerk
    refPositionWderX = addZeros(refPointsX, [1, 1, lenRef, lenRef]);
    refPositionWderY = addZeros(refPointsY, [1, 1, lenRef, lenRef]);
    refPositionWderZ = addZeros(refPointsZ, [1, 1, lenRef, lenRef]);
    
    dSplTimeRef = addZeros(splTimeRef, [1, 1]);
    lenTime = length(dSplTimeRef);
    dSplTimeRef = addValueAt(dSplTimeRef, [lenTime, lenTime], dSplTimeRef(end));
else
    refPositionWderX = addZeros(refPointsX, [1, 1, 1, lenRef, lenRef, lenRef]);
    refPositionWderY = addZeros(refPointsY, [1, 1, 1, lenRef, lenRef, lenRef]);
    refPositionWderZ = addZeros(refPointsZ, [1, 1, 1, lenRef, lenRef, lenRef]);
    
    dSplTimeRef = addZeros(splTimeRef, [1, 1, 1]);
    lenTime = length(dSplTimeRef);
    dSplTimeRef = addValueAt(dSplTimeRef, [lenTime, lenTime, lenTime], dSplTimeRef(end));
end

% Create curves
knots = optknt(dSplTimeRef, order);
hSplCurveX = spapi(knots, dSplTimeRef, refPositionWderX);
hSplPointsX = fnval(time, hSplCurveX);

hSplCurveY = spapi(knots, dSplTimeRef, refPositionWderY);
hSplPointsY = fnval(time, hSplCurveY);

hSplCurveZ = spapi(knots, dSplTimeRef, refPositionWderZ);
hSplPointsZ = fnval(time, hSplCurveZ);

newSplineXYZ = [hSplPointsX ; hSplPointsY ; hSplPointsZ];

[curvesX, curvesY, curvesZ, linCurves] = newComputeDiff(time, hSplPointsX, hSplPointsY, hSplPointsZ);

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
        [newTime, newSplineXYZ] = stretch3DCurve(refTime, factor^i, lenRef, ...
            [refPositionWderX ; refPositionWderY ; refPositionWderZ], sampleFreq, order, zeroJerk);

        linDistanceOutput = sqrt(sum([newSplineXYZ(1,1:end); newSplineXYZ(2,1:end); newSplineXYZ(3,1:end);].^2));
        linSpeedOutput = diff(linDistanceOutput)./diff(newTime);
        linAccelOutput = diff(linSpeedOutput)./diff(newTime(1:end-1));
        linJerkOutput = diff(linAccelOutput)./diff(newTime(1:end-2));

        failIndexSpeed = checkConstraints(linSpeedOutput, maxLinSpeed);
        failIndexAccel = checkConstraints(linAccelOutput, maxLinAccel);
        failIndexJerk = checkConstraints(linJerkOutput, maxLinJerk);

        i = i+1;
        continue
    end
    doneStretching = true;
    fprintf('\nStretched time %d times, by a total factor of %f\n', i-1, factor^(i-1))
    fprintf('Original duration of trajectory: %f s\nNew duration: %f s\n', refTime, newTime(end))
end

%% Save CSV file
% Reference
refPointsData = array2table([splTimeRef' refPointsX' refPointsY' refPointsZ'], 'VariableNames', {'time', 'x', 'y', 'z'});
writetable(refPointsData, "data/3DrefPoints.csv", 'WriteVariableNames', true);

refSpeedData = array2table([time(1:end-1)' linSpeedRef'], 'VariableNames', {'time', 'speed'});
writetable(refSpeedData, "data/3DrefLinSpeed.csv", 'WriteVariableNames', true);

refAccelData = array2table([time(1:end-2)' linAccelRef'], 'VariableNames', {'time', 'accel'});
writetable(refAccelData, "data/3DrefLinAccel.csv", 'WriteVariableNames', true);

% Output
outputPosition = array2table([newSplineXYZ(1,1:end)' newSplineXYZ(2,1:end)' newSplineXYZ(3,1:end)'], 'VariableNames', {'x', 'y', 'z'});
writetable(outputPosition, "data/3DoutputXYZ.csv", 'WriteVariableNames', true);

outputLinSpeed = array2table([newTime(1:end-1)' linSpeedOutput'], 'VariableNames', {'time', 'speed'});
writetable(outputLinSpeed, "data/3DoutputSpeed.csv", 'WriteVariableNames', true);

outputLinAccel = array2table([newTime(1:end-2)' linAccelOutput'], 'VariableNames', {'time', 'accel'});
writetable(outputLinAccel, "data/3DoutputAccel.csv", 'WriteVariableNames', true);

outputLinJerk = array2table([newTime(1:end-3)' linJerkOutput'], 'VariableNames', {'time', 'jerk'});
writetable(outputLinJerk, "data/3DoutputJerk.csv", 'WriteVariableNames', true);

%% Reference Plot
figure('NumberTitle', 'off', 'Name', 'Reference Values');
subplot(3,1,1)
plot(time, linDistanceRef, '-r', 'LineWidth', 1.5)
hold on
grid on
ylabel('Position [m]')
title('Reference')

subplot(3,1,2)
plot(time(1:end-1), linSpeedRef, '-b', 'LineWidth', 1.5)
hold on
plot(time, maxLinSpeed*ones(size(time)), '--k', 'LineWidth', 1.0)
plot(time, -maxLinSpeed*ones(size(time)), '--k', 'LineWidth', 1.0)
grid on
ylabel('Speed [m/s]')

subplot(3,1,3)
plot(time(1:end-2), linAccelRef, '-g', 'LineWidth', 1.5)
hold on
plot(time, maxLinAccel*ones(size(time)), '--k', 'LineWidth', 1.0)
plot(time, -maxLinAccel*ones(size(time)), '--k', 'LineWidth', 1.0)
grid on
ylabel('Acceleration [m/s^2]')
xlabel('Time [s]')

%% Trajectory and Linear Plots
figure('NumberTitle', 'off', 'Name', 'High order 3D Fitting - XYZ Space');

hold on
grid on
plot3(newSplineXYZ(1,1:end), newSplineXYZ(2,1:end), newSplineXYZ(3,1:end), '-r', 'LineWidth', 1.5)
plot3(refPointsX, refPointsY, refPointsZ, '-r*', 'LineWidth', 1.5, 'LineStyle', 'none')
legend({'Position'; 'Ref points'})
xlabel('X-axis [m]')
ylabel('Y-axis [m]')
zlabel('Z-axis [m]')
title('Position in XYZ Space')


figure('NumberTitle', 'off', 'Name', 'High order 3D Fitting - Linear Derivatives');

subplot(3, 1, 1)
hold on
grid on
plot(newTime(1:end-1), linSpeedOutput, '-b', 'LineWidth', 1.5)
plot(newTime, maxLinSpeed*ones(1, length(newTime)), '--k')
plot(newTime, -maxLinSpeed*ones(1, length(newTime)), '--k')
legend({'Speed', 'Limits'})
ylabel('Speed [m/s]')
title('Linear Derivatives')

subplot(3, 1, 2)
hold on
grid on
plot(newTime(1:end-2), linAccelOutput, '-m', 'LineWidth', 1.5)
plot(newTime, maxLinAccel*ones(1, length(newTime)), '--k')
plot(newTime, -maxLinAccel*ones(1, length(newTime)), '--k')
legend({'Acceleration', 'Limits'})
ylabel('Acceleration [m/s^2]')

subplot(3, 1, 3)
hold on
grid on
plot(newTime(1:end-3), linJerkOutput, '-g', 'LineWidth', 1.5)
plot(newTime, maxLinJerk*ones(1, length(newTime)), '--k')
plot(newTime, -maxLinJerk*ones(1, length(newTime)), '--k')
legend({'Jerk', 'Limits'})
xlabel('Time [s]')
ylabel('Jerk [m/s^3]')
