function [newTime, newSplineXY] = stretch2DCurve(refTime, factor, lenRef, refPointsWder, sampleFreq, order)
%stretch2DCurve       Stretch 2D curve in time.
%     newTime, newSplineXY = stretch2DCurve(t,f,p,dp,s,o) returns the new time vector
%     created after the stretching process and the points in the new spline curves,
%     in both dimensions of p. t is the original duration of the trajectory, used to
%     generate newTime, which is upper bounded by t*f, and has s*t*f points. The new
%     spline points are calculated using newTime. The interpolation is of k-th order
%     and the new curve also considers the derivatives of the reference points as
%     indicated by dp.
%     The new points of the curve's first and second dimensions are 
%           newSplineXY(1,1:end) and newSplineXY(2,1:end), respectively.

    newTimeRef = linspace(0, refTime*factor, lenRef);
    newTime = linspace(0, refTime*factor, sampleFreq*refTime*factor);
    
    dSplTimeRef = addValueAt(newTimeRef, [1, 1], 0);
    lenTime = length(dSplTimeRef);
    dSplTimeRef = addValueAt(dSplTimeRef, [lenTime, lenTime], dSplTimeRef(end));
    
    hSplCurveX = spapi(optknt(dSplTimeRef, order), dSplTimeRef, refPointsWder(1,1:end));
    hSplCurveY = spapi(optknt(dSplTimeRef, order), dSplTimeRef, refPointsWder(2,1:end));

    newSplinePointsX = fnval(newTime, hSplCurveX);
    newSplinePointsY = fnval(newTime, hSplCurveY);
    
    newSplineXY = [newSplinePointsX ; newSplinePointsY];
end
