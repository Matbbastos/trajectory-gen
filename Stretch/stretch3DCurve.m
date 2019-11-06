function [newTime, newSplineXYZ] = stretch3DCurve(refTime, factor, lenRef, refPointsWder, sampleFreq, order)
%stretch3DCurve       Stretch 3D curve in time.
%     newTime, newSplineXY = stretch3DCurve(t,f,l,dp,s,o) returns the new time vector
%     created after the stretching process and the points in the new spline curves,
%     in all three dimensions of dp. t is the original duration of the trajectory, used 
%     to generate newTime, which is upper bounded by t*f, and has s*t*f points. The new
%     spline points are calculated using newTime. The interpolation is of k-th order
%     and the new curve also considers the derivatives of the reference points as
%     indicated by repeated entries in dp.
%     The new points of the curve's first and second dimensions are, respectively
%           newSplineXYZ(1,1:end) and newSplineXYZ(2,1:end) and newSplineXYZ(3,1:end).
%     See also stretch1DCurve, stretch2DCurve 

    newTimeRef = linspace(0, refTime*factor, lenRef);
    newTime = linspace(0, refTime*factor, sampleFreq*refTime*factor);
    
    dSplTimeRef = addValueAt(newTimeRef, [1, 1], 0);
    lenTime = length(dSplTimeRef);
    dSplTimeRef = addValueAt(dSplTimeRef, [lenTime, lenTime], dSplTimeRef(end));
    
    hSplCurveX = spapi(optknt(dSplTimeRef, order), dSplTimeRef, refPointsWder(1,1:end));
    hSplCurveY = spapi(optknt(dSplTimeRef, order), dSplTimeRef, refPointsWder(2,1:end));
    hSplCurveZ = spapi(optknt(dSplTimeRef, order), dSplTimeRef, refPointsWder(3,1:end));

    newSplinePointsX = fnval(newTime, hSplCurveX);
    newSplinePointsY = fnval(newTime, hSplCurveY);
    newSplinePointsZ = fnval(newTime, hSplCurveZ);
    
    newSplineXYZ = [newSplinePointsX ; newSplinePointsY ; newSplinePointsZ];
end
