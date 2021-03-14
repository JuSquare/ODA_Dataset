function [text] = printMatrixAsCArray(A,maxElementsPerLine)
%PRINTASCARRAY Summary of this function goes here
%   Detailed explanation goes here

text  = '';

S = size(A);

if (length(S) > 2)
    error('Input must be a 2D matrix.')
end
ii = S==1;
SS = S(~ii);

if (isempty(SS))
    error('Input cannot be a scalar.')
end
    
if (length(SS) == 1)
    if (~exist('maxElementsPerLine','var'))
        maxElementsPerLine = SS;
    end
    fprintf('{');
    for i = 1:SS
        fprintf('%3.4f,',A(i));
    end
    fprintf('%c};\n',8);
end

if (length(SS) == 2)
    if (~exist('maxElementsPerLine','var'))
        maxElementsPerLine = SS(2);
    end
    fprintf('{');
    for i = 1:SS(1)
        fprintf('{');
        for j = 1:SS(2)
            fprintf('%3.4f,',A(i,j));
            %if (mod(j,maxElementsPerLine) == 0 && j < SS(2))
            %    fprintf('\n');
            %end
        end
        fprintf('%c},\n',8);
    end
    fprintf('%c%c};\n',8,8);
end

end

