% FILENAME: genDCM.m
% FILETYPE: function
% DESCRIPTION: genDCM produces a Direction Cosine Matrix (DCM) using user
% specified units, rotation angles, and rotation sequence
% 
% INPUTS:
%   - units: The units ('deg' or ' rad') of the provided rotation angles
%   - rotations: Angles that correspond to provided rotation sequence
%   - sequence: Order of rotation corresponding to the 3 axes (1,2, or 3)
%           where x is normally 1. y is normally 2, and z is normally 3
% OUTPUTS:
%   - DCM: Matrix that is used to bring a vector to another frame
%         EX: V_b = DCM*V_a
%
% AUTHOR(S): Noah Miller (nsm0014@auburn.edu)
% DATE: 9/15/2022

function DCM = genDCM(units,Rotations,Sequence)

n = length(Sequence);
Matrix = zeros([3 3 n]);

% Creating DCM based on units specified from user input
if strcmp('deg',units)
    for i = 1:n
        k = Sequence(i);
        switch k
            case 1
                Matrix(:,:,i) = [1        0                  0           ;...
                                 0 cosd(Rotations(i)) -sind(Rotations(i));...
                                 0 sind(Rotations(i)) cosd(Rotations(i))];
            case 2
                Matrix(:,:,i) = [cosd(Rotations(i)) 0 sind(Rotations(i));...
                                 0                  1        0          ;...
                                -sind(Rotations(i)) 0 cosd(Rotations(i))];
            case 3
                Matrix(:,:,i) = [cosd(Rotations(i)) -sind(Rotations(i)) 0;...
                                 sind(Rotations(i)) cosd(Rotations(i))  0;...
                                       0                  0             1];
        end
    end
end

if strcmp('rad',units)
    for i = 1:n
        k = Sequence(i);
        switch k
            case 1
                Matrix(:,:,i) = [1        0                  0           ;...
                                 0 cos(Rotations(i)) -sin(Rotations(i));...
                                 0 sin(Rotations(i)) cos(Rotations(i))];
            case 2
                Matrix(:,:,i) = [cos(Rotations(i)) 0 sin(Rotations(i));...
                                 0                 1        0         ;...
                                -sin(Rotations(i)) 0 cos(Rotations(i))];
            case 3
                Matrix(:,:,i) = [cos(Rotations(i)) -sin(Rotations(i)) 0;...
                                 sin(Rotations(i)) cos(Rotations(i))  0;...
                                       0                  0           1];
        end
    end
end

% Multiplying n number of rotations into a single DCM
DCM = Matrix(:,:,1);
for i = 2:n

    DCM = (DCM*Matrix(:,:,i));

end
DCM = DCM';
end