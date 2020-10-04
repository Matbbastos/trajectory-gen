addpath('Stretch');

%% Initial settings
refPoints = [0 2 15 10 22 -1 -8 2 10 0];   % [m] position in X-axis
lenRef = length(refPoints);

maxLinSpeed = 7;    % [m/s]
maxLinAccel = 10;   % [m/s^2]
maxLinJerk = 20;    % [m/s^3]
sampleFreq = 100;   % [Hz]

refTime = 0;    % [s] sum of time to reach each point in a straight line from the previous point

for i = 1:(lenRef - 1)
    refTime = refTime + abs((refPoints(i+1) - refPoints(i))/maxLinSpeed);
end

%% High order spline with derivatives and stretching
order = 6;
time = linspace(0, refTime, sampleFreq*refTime);
splTimeRef = linspace(0, refTime, lenRef);

% Reference scenario
refDistance = interp1(splTimeRef, refPoints, time);
speedRef = diff(refDistance)./diff(time);
accelRef = diff(speedRef)./diff(time(1:end-1));

% Add conditions to derivatives (speed = acceleration = 0 at start and end)
refPositionWder = addZeros(refPoints, [1, 1, lenRef, lenRef]);
dSplTimeRef = addZeros(splTimeRef, [1, 1]);
lenTime = length(dSplTimeRef);
dSplTimeRef = addValueAt(dSplTimeRef, [lenTime, lenTime], dSplTimeRef(end));

% Create curves
knots = optknt(dSplTimeRef, order);
hSplCurve = spapi(knots, dSplTimeRef, refPositionWder);
hSplPoints = fnval(time, hSplCurve);

hSplSpeed = diff(hSplPoints)./diff(time);
hSplAccel = diff(hSplSpeed)./diff(time(1,1:end-1));
hSplJerk = diff(hSplAccel)./diff(time(1,1:end-2));

% Check constraints
failIndexSpeed = checkConstraints(hSplSpeed, maxLinSpeed);
failIndexAccel = checkConstraints(hSplAccel, maxLinAccel);
failIndexJerk = checkConstraints(hSplJerk, maxLinJerk);

% Stretch curve if necessary
factor = 1.05;
doneStretching = false;
i = 0;

while ~doneStretching
    if ~isempty(failIndexSpeed) || ~isempty(failIndexAccel) || ~isempty(failIndexJerk)
        [newTime, newPoints] = stretch1DCurve(refTime, factor^i, lenRef, refPositionWder, sampleFreq, order);
        hSplSpeed = diff(newPoints)./diff(newTime);
        hSplAccel = diff(hSplSpeed)./diff(newTime(1:end-1));
        hSplJerk = diff(hSplAccel)./diff(newTime(1:end-2));
        failIndexSpeed = checkConstraints(hSplSpeed, maxLinSpeed);
        failIndexAccel = checkConstraints(hSplAccel, maxLinAccel);
        failIndexJerk = checkConstraints(hSplJerk, maxLinJerk);
        i = i+1;
        continue
    end
    doneStretching = true;
    fprintf('\nStretched time %d times, by a total factor of %f\n', i-1, factor^(i-1))
    fprintf('Original duration of trajectory: %f s\nNew duration: %f s\n', refTime, newTime(end))
end

%% Save CSV files
% Reference
refDistanceData = array2table([time' refDistance'], 'VariableNames', {'time', 'position'});
writetable(refDistanceData, "data/1DrefDistance.csv", 'WriteVariableNames', true);

refPointsData = array2table([splTimeRef' refPoints'], 'VariableNames', {'time', 'points'});
writetable(refPointsData, "data/1DrefPoints.csv", 'WriteVariableNames', true);

refSpeedData = array2table([time(1:end-1)' speedRef'], 'VariableNames', {'time', 'speed'});
writetable(refSpeedData, "data/1DrefSpeed.csv", 'WriteVariableNames', true);

refAccelData = array2table([time(1:end-2)' accelRef'], 'VariableNames', {'time', 'accel'});
writetable(refAccelData, "data/1DrefAccel.csv", 'WriteVariableNames', true);

% Output
outputPos = array2table([newTime' newPoints'], 'VariableNames', {'time', 'position'});
writetable(outputPos, "data/1DoutputPos.csv", 'WriteVariableNames', true);

forOutputPoints = array2table([linspace(0, newTime(end), lenRef)' refPoints'], 'VariableNames', {'time', 'points'});
writetable(forOutputPoints, "data/1DforOutputPoints.csv", 'WriteVariableNames', true);

outputSpeed = array2table([newTime(1:end-1)' hSplSpeed'], 'VariableNames', {'time', 'speed'});
writetable(outputSpeed, "data/1DoutputSpeed.csv", 'WriteVariableNames', true);

outputAccel = array2table([newTime(1:end-2)' hSplAccel'], 'VariableNames', {'time', 'accel'});
writetable(outputAccel, "data/1DoutputAccel.csv", 'WriteVariableNames', true);

outputSJerk = array2table([newTime(1:end-3)' hSplJerk'], 'VariableNames', {'time', 'jerk'});
writetable(outputSJerk, "data/1DoutputJerk.csv", 'WriteVariableNames', true);

%% Reference Scenario
figure('NumberTitle', 'off', 'Name', 'Reference Values');
subplot(3,1,1)
plot(time, refDistance, '-r', 'LineWidth', 1.5)
hold on
grid on
plot(splTimeRef, refPoints, 'o',...
    'MarkerSize', 5, 'MarkerEdgeColor','red',...
    'MarkerFaceColor', [1 .6 .6])
ylabel('Position [m]')
title('Reference')

subplot(3,1,2)
plot(time(1:end-1), speedRef, '-b', 'LineWidth', 1.5)
hold on
plot(time, maxLinSpeed*ones(size(time)), '--k', 'LineWidth', 1.0)
plot(time, -maxLinSpeed*ones(size(time)), '--k', 'LineWidth', 1.0)
grid on
ylabel('Speed [m/s]')

subplot(3,1,3)
plot(time(1:end-2), accelRef, '-g', 'LineWidth', 1.5)
hold on
plot(time, maxLinAccel*ones(size(time)), '--k', 'LineWidth', 1.0)
plot(time, -maxLinAccel*ones(size(time)), '--k', 'LineWidth', 1.0)
grid on
ylabel('Acceleration [m/s^2]')
xlabel('Time [s]')

%% Output Plots
figure('NumberTitle', 'off', 'Name', 'Time Stretching with Spline');

subplot(2, 1, 1)
hold on
grid on
plot(newTime, newPoints, '-r', 'LineWidth', 1.5)
plot(linspace(0, newTime(end), lenRef), refPoints, '-r*', 'LineWidth', 1.5, 'LineStyle', 'none')
legend({'Position'; 'Ref points'})
ylabel('Distance [m]')
title('Output')

subplot(2, 1, 2)
hold on
grid on
plot(newTime(1:end-1), hSplSpeed, '-b', 'LineWidth', 1.5)
plot(newTime, maxLinSpeed*ones(1, length(newTime)), '--k')
plot(newTime, -maxLinSpeed*ones(1, length(newTime)), '--k')
legend({'Speed', 'Limits'})
ylabel('Speed [m/s]')
xlabel('Time [s]')

figure('NumberTitle', 'off', 'Name', 'Time Stretching with Spline');

subplot(2, 1, 1)
hold on
grid on
plot(newTime(1:end-2), hSplAccel, '-m', 'LineWidth', 1.5)
plot(newTime, maxLinAccel*ones(1, length(newTime)), '--k')
plot(newTime, -maxLinAccel*ones(1, length(newTime)), '--k')
legend({'Acceleration', 'Limits'})
title('Output')
ylabel('Acceleration [m/s^2]')

subplot(2, 1, 2)
hold on
grid on
plot(newTime(1:end-3), hSplJerk, '-g', 'LineWidth', 1.5)
plot(newTime, maxLinJerk*ones(1, length(newTime)), '--k')
plot(newTime, -maxLinJerk*ones(1, length(newTime)), '--k')
legend({'Jerk', 'Limits'})
xlabel('Time [s]')
ylabel('Jerk [m/s^3]')

%% Try local stretch
% currently working with only one point. If there is more than one point to
% stretch, the indexes do not behave as desired. Also, there is no
% implementation on how to get the indexes, it is manually as of now.
% lenTime = length(time);
% 
% indexFailStart = 150;
% indexFailEnd = 200;
% 
% periodFailStart = indexFailStart*time(end)/lenTime;
% periodFailEnd = indexFailEnd*time(end)/lenTime;
% 
% pointsTimeFail = find((splTimeRef >= periodFailStart) & (splTimeRef <= periodFailEnd));
% pointsToAdd = zeros(length(pointsTimeFail), 3);
% stepSize = (splTimeRef(2) - splTimeRef(1))/2;
% 
% refPointsCopy = refPoints;
% for i = 1:length(pointsTimeFail)
%     pointsToAdd(i, 1) = fnval(splTimeRef(pointsTimeFail(i)) - stepSize, hSplCurve);
%     pointsToAdd(i, 2) = refPointsCopy(pointsTimeFail(i));
%     pointsToAdd(i, 3) = fnval(splTimeRef(pointsTimeFail(i)) + stepSize, hSplCurve);
% end
% 
% for i = 1:length(pointsTimeFail)
%     newRefPoints = [refPointsCopy(1:pointsTimeFail(i)-1) pointsToAdd(i,1:end) refPointsCopy(pointsTimeFail(i)+1:end)];
%     refPointsCopy = newRefPoints;
% end
% 
% limit = round(refTime(end)+length(pointsTimeFail)*stepSize);
% novoTime = linspace(0, limit, sampleFreq*limit);
% nSplTimeRef = linspace(0, limit, length(newRefPoints));
% nHSplCurve = spapi(optknt(nSplTimeRef, order), nSplTimeRef, newRefPoints);
% nHSplPoints = fnval(novoTime, nHSplCurve);
% nHSplSpeed = fnval(novoTime, fnder(nHSplCurve));

% Plots
% f = figure('NumberTitle', 'off', 'Name', 'High order splinetTest');
% f.WindowState = 'maximized';
% 
% subplot(2, 1, 1)
% hold on
% grid on
% plot(time, hSplPoints, '-b', 'LineWidth', 1.5)
% plot(splTimeRef, refPoints, '-b^', 'LineWidth', 1.5, 'LineStyle', 'none')
% plot(novoTime, nHSplPoints, '-r', 'LineWidth', 1.5)
% plot(nSplTimeRef, newRefPoints, '-r^', 'LineWidth', 1.5, 'LineStyle', 'none')
% legend({'Original Position'; 'Original Ref points'; 'New Position'; 'New Ref points'})
% ylabel('Distance [m]')
% title('X-Axis')
% 
% subplot(2, 1, 2)
% hold on
% grid on
% plot(time, hSplSpeed, '-b', 'LineWidth', 1.5)
% plot(novoTime, nHSplSpeed, '-r', 'LineWidth', 1.5)
% legend({'Original Speed'; 'New Speed'})
% ylabel('Speed [m/s]')
