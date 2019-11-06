function varargout = computeDiff(varargin)
%computeDiff       Compute up to 3rd derivatives in X and Y-axis and linear derivatives.
%     [cx, cy, lc] = computeDiff(t,px,py) returns the computation of the derivatives
%     up to the 3rd order, in respect to time vector t, of the curves specified by 
%     px and py, and also the linear derivatives, considering 
%           lc{j}(i) = norm([px{j}(i) py{j}(i)]), where
%       1<=j<=3 indicates the order of the derivative, and
%       1<=i<=length(time) the index of the point on the curve
%     cx, cy and lc have size (3, length(time)), each line indicating one
%     derivative. The second and third derivatives' vectors are right padded with
%     zeros to fit in the same array as the first derivative.

    time = varargin{1};
    hSplPointsX = varargin{2};
    hSplPointsY = varargin{3};

    % Position, speed, acceleration and jerk on X-axis
    hSplSpeedX = diff(hSplPointsX)./diff(time);
    hSplAccelX = diff(hSplSpeedX)./diff(time(1,1:end-1));
    hSplJerkX = diff(hSplAccelX)./diff(time(1,1:end-2));

    % Position, speed, acceleration and jerk on Y-axis
    hSplSpeedY = diff(hSplPointsY)./diff(time);
    hSplAccelY = diff(hSplSpeedY)./diff(time(1,1:end-1));
    hSplJerkY = diff(hSplAccelY)./diff(time(1,1:end-2));

    % Linear speed, acceleration and jerk
    linSpeed = zeros(1, length(hSplSpeedX));
    linAccel = zeros(1, length(hSplAccelX));
    linJerk = zeros(1, length(hSplJerkX));

        
    if nargin == 3
        for i = 1:length(hSplSpeedX)
            linSpeed(i) = norm([hSplSpeedX(i) hSplSpeedY(i)]);
            if i <= length(hSplSpeedX)-1 
                linAccel(i) = norm([hSplAccelX(i) hSplAccelY(i)]);
                if i <= length(hSplSpeedX)-2 
                    linJerk(i) = norm([hSplJerkX(i) hSplJerkY(i)]);
                end
            end
        end

        varargout{1} = [hSplSpeedX ; [hSplAccelX 0]; [hSplJerkX 0 0]];
        varargout{2} = [hSplSpeedY ; [hSplAccelY 0]; [hSplJerkY 0 0]];
        varargout{3} = [linSpeed ; [linAccel 0]; [linJerk 0 0]];

    elseif nargin == 4
        hSplPointsZ = varargin{4};
        
        % Position, speed, acceleration and jerk on Z-axis
        hSplSpeedZ = diff(hSplPointsZ)./diff(time);
        hSplAccelZ = diff(hSplSpeedZ)./diff(time(1,1:end-1));
        hSplJerkZ = diff(hSplAccelZ)./diff(time(1,1:end-2));

        for i = 1:length(hSplSpeedX)
            linSpeed(i) = norm([hSplSpeedX(i) hSplSpeedY(i) hSplJerkZ(i)]);
            if i <= length(hSplSpeedX)-1 
                linAccel(i) = norm([hSplAccelX(i) hSplAccelY(i) hSplJerkZ(i)]);
                if i <= length(hSplSpeedX)-2 
                    linJerk(i) = norm([hSplJerkX(i) hSplJerkY(i) hSplJerkZ(i)]);
                end
            end
        end

        varargout{1} = [hSplSpeedX ; [hSplAccelX 0]; [hSplJerkX 0 0]];
        varargout{2} = [hSplSpeedY ; [hSplAccelY 0]; [hSplJerkY 0 0]];
        varargout{3} = [hSplSpeedZ ; [hSplAccelZ 0]; [hSplJerkZ 0 0]];
        varargout{4} = [linSpeed ; [linAccel 0]; [linJerk 0 0]];
    end
end
