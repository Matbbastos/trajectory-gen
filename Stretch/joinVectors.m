function newVector = joinVectors(varargin)
%joinVectors        Joins vectors of same length, ordered by index.
%     The function can be called using any number of vectors as input, the reference
%     for length is the first vector. Let N be the number of inputs and p{j} with
%     1<=j<=N being the j-th input, the new vector V is created using
%           V(N*i+j) = p{j}(i+1)
%     for i = 0:length(p{1}) - 1
%     
%     Example with 3 vectors:
%     	V = joinVectors(a,b,c) returns a vector where 
%           V(3*i+1) = a(i+1)
%           V(3*i+2) = b(i+1)
%           V(3*i+3) = c(i+1)
%     	for i = 0:length(a) - 1

    msgID = 'joinVectors:BadLengths';
    msg = 'Vectors don''t have the same length.';
    baseException = MException(msgID, msg);
    
    refLen = length(varargin{1});
    for i = 2:nargin
        if length(varargin{i}) ~= refLen
            causeException = MException('MATLAB:unmatchedLengths',sprintf('Input %d has incompatible length (reference is 1st input).', i));
            baseException = addCause(baseException,causeException);
            throw(baseException);
        end
    end

    newVector = zeros(1, nargin*refLen);    
    for i = 0:refLen - 1
        for j = 1:nargin
            newVector(nargin*i+j) = varargin{j}(i+1);
        end
    end
end