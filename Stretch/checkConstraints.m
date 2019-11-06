function failIndexes = checkConstraints(curve, limit)
%checkConstraints       Check if curve is bounded by limit.
%     I = checkConstraints(c,l) returns a vector containing the indexes in which the
%     absolute value of the curve c is above l.

    failIndexes = zeros(length(curve));
    currentI = 1;
    for i = 1:length(curve)
        if abs(curve(i)) > limit
            failIndexes(currentI) = i;
            currentI = currentI + 1;
        end
    end
    failIndexes(failIndexes == 0) = [];
end
