function newVector = addValueAt(vector, indexes, value)
%addValueAt        Adds value to the desired original indexes.
%     addValueAt(a,i,k) Returns a vector V, using a vector a, an array of indexes i
%     and a value k as input. The returned vector V is such that 
%           length(V) = length(a) + length(i).
%     V has the same values of a, except that in each index present in i, the value 
%     k is inserted, shifting the values to right.
%     
%     Example:
%     	V = addZeros([1, 2, 3], [1, 1, 3], -5) returns a vector 
%           V = [1, -5, -5, 2, 3, -5]
%     
%       See also addZeros

    indexes = sort(indexes);
    newVector = value*ones(1, length(vector) + length(indexes));
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