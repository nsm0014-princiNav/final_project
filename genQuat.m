% FILENAME: genQuat.m
% FILETYPE: function
% DESCRIPTION: genQuat produces a quaternion array using k and theta
% provided by the user
%
% INPUTS:
%   - units: The units ('deg' or ' rad') of the provided rotation angles
%   - theta_deg: magnitude of rotation about axis
%   - k: unit vector of axis to rotate about
%
% OUTPUTS:
%   - q: 4 x 1 quaternion array in the order:
%           1. scalar
%           2. 1st axis
%           3. 2nd axis
%           4. 3rd axis
%   - inv_q: q* or inverse of q
%
% AUTHOR(S): Noah Miller (nsm0014@auburn.edu)
% DATE: 9/15/2022

function [q,inv_q] = genQuat(units,theta,k)

if strcmp('deg',units)

    q0 = cosd(theta/2);
    q1 = k(1)*sind(theta/2);
    q2 = k(2)*sind(theta/2);
    q3 = k(3)*sind(theta/2);

else

    q0 = cos(theta/2);
    q1 = k(1)*sin(theta/2);
    q2 = k(2)*sin(theta/2);
    q3 = k(3)*sin(theta/2);

end
q = [q0;q1;q2;q3];
inv_q = [q0;-q1;-q2;-q3]; 
end