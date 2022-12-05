% FILENAME: formskewsym_inv.m
% FILETYPE: function
% DESCRIPTION: formskewsym_inv produces a 3 row, 1 column X vector
% from a 3 row, 3 column skew-symmetric matrix 
% 
% INPUTS:
%   - skew_X: 3 row, 3 column skew symmetric Matrix

% OUTPUTS:
%   - X: 3 rows, 1 column vector
%
% AUTHOR(S): Noah Miller (nsm0014@auburn.edu)
% DATE: 10/22/2022 

function X = formskewsym_inv(skew_X)

X = [skew_X(3,2);skew_X(1,3);skew_X(2,1)];

end