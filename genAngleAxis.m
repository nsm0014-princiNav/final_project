% FILENAME: genAngleAxis.m
% FILETYPE: function
% DESCRIPTION: genAngleAxis produces theta in both radians and degrees and
% k using a Direction Cosine Matrix (DCM)
% 
% INPUTS:
%   - DCM: Matrix that is used to bring a vector to another frame
%         EX: V_b = DCM*V_a
% OUTPUTS:
%   - theta_deg: magnitude of rotation about axis in degrees
%   - theta_rad: magnitude of rotation about axis in radians
%   - k: unit vector of axis to rotate about
%
% AUTHOR(S): Noah Miller (nsm0014@auburn.edu)
% DATE: 9/15/2022

function [theta_deg,theta_rad,k] = genAngleAxis(DCM)

theta_deg = acosd((trace(DCM)-1)/2);
theta_rad = acos((trace(DCM)-1)/2);

k = [(DCM(3,2) - DCM(2,3));(DCM(1,3) - DCM(3,1));(DCM(2,1) - DCM(1,2))]./(2*sind(theta_deg));

end