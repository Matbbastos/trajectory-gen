function newVector = addZeros(vector, indexes)
%addZeros        Adds zeros to the desired original indexes.
%     addZeros(a,i) Returns a vector V, using a vector a and an array of indexes i 
%     as input. The returned vector V is such that 
%           length(V) = length(a) + length(i).
%     V has the same values of a, except that in each index present in i, a zero is 
%     inserted, shifting the values to right.
%     
%     Example:
%     	V = addZeros([1, 2, 3], [1, 1, 3]) returns a vector 
%           V = [1, 0, 0, 2, 3, 0]
%     
%       See also addValueAt

    indexes = sort(indexes);
    newVector = zeros(1, length(vector) + length(indexes));
    j = 0;

    for i = 1:length(vector)
        if ismember(i, indexes)
            newVector(i+j) = vector(i);
            for k = 1:sum(indexes == i)
                if i+j+k <= length(newVector)
                    j = j + 1;
                end
            end
        else
            newVector(i+j) = vector(i);
        end
    end
end