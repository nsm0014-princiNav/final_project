% FILENAME: formskewsym.m
% FILETYPE: function
% DESCRIPTION: formskewsym produces a 3 row, 3 column skew-symmetric matrix
% from input X 
% 
% INPUTS:
%   - X: 3 rows, 1 column vector
% OUTPUTS:
%   - skew_X: 3 row, 3 column skew symmetric Matrix
%
% AUTHOR(S): Noah Miller (nsm0014@auburn.edu)
% DATE: 10/22/2022 

function skew_X = formskewsym(X)

skew_X = [0, -1*X(3), X(2);X(3), 0, -1*X(1);-1*X(2) X(1) 0];

end
