function [newTime, newSplinePoints] = stretch1DCurve(refTime, factor, lenRef, refPointsWder, sampleFreq, order)
%stretch1DCurve       Stretch 1D curve in time.
%     newTime, newSplineXY = stretch1DCurve(t,f,l,dp,s,k) returns the new time vector
%     created after the stretching process and the points in the new spline curves,
%     in dp. t is the original duration of the trajectory, used to generate newTime,
%     which is upper bounded by t*f, and has s*t*f points. The new spline points are
%     calculated using newTime. The interpolation is of k-th order and the new curve
%     also considers the derivatives of the reference points as indicated by repeated
%     entries in dp.
%     See also stretch2DCurve, stretch3DCurve 

    newTimeRef = linspace(0, refTime*factor, lenRef);
    newTime = linspace(0, refTime*factor, sampleFreq*refTime*factor);
    
    dSplTimeRef = addValueAt(newTimeRef, [1, 1], 0);
    lenTime = length(dSplTimeRef);
    dSplTimeRef = addValueAt(dSplTimeRef, [lenTime, lenTime], dSplTimeRef(end));
    
    hSplCurve = spapi(optknt(dSplTimeRef, order), dSplTimeRef, refPointsWder);
    newSplinePoints = fnval(newTime, hSplCurve);
end
