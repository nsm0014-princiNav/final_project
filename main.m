%% Formatting
clc
clear
close all
format shortg
addpath("data\")
%% Loading in .mat files from Truck data
load("random.mat","random")
load("static.mat","static")
load("turn.mat","turn")

% Testing turn case
old_PVA(1).pos = (lla2ecef([32.59836159854743, -85.29694919181105, 200]))';
old_PVA(1).vel = [0 0 0]';
old_PVA(1).C_b2e = diag([1 1 1]);
dt = diff(turn.IMU.Time);

for i = 1:length(turn.IMU.Time) - 1

    [p_eb_e, v_eb_e, C_b2e] = imu.Mechanize_ECEF([turn.IMU.Linear_Acceleration(i,1) turn.IMU.Linear_Acceleration(i,3) turn.IMU.Linear_Acceleration(i,2)]', [turn.IMU.Angular_Velocity(i,1) turn.IMU.Angular_Velocity(i,3) turn.IMU.Angular_Velocity(i,2)]', dt(i), old_PVA(i));

    old_PVA(i+1).pos = p_eb_e;
    old_PVA(i+1).vel = v_eb_e;
    old_PVA(i+1).C_b2e = C_b2e;

    roll(i) = atan2d(-C_b2e(2,3),C_b2e(3,3));
    pitch(i) = atan2d(C_b2e(1,3),(sqrt(C_b2e(2,3)^2 + C_b2e(3,3)^2)));
    yaw(i) = atan2d(-C_b2e(2,1),C_b2e(1,1));
end

figure
tiledlayout(3,1)
nexttile
plot(turn.IMU.Time,turn.IMU.Angular_Velocity(:,1)*(180/pi))
nexttile
plot(turn.IMU.Time,turn.IMU.Angular_Velocity(:,3)*(180/pi))
nexttile
plot(turn.IMU.Time,turn.IMU.Angular_Velocity(:,2)*(180/pi))

figure
tiledlayout(3,1)
nexttile
plot(turn.IMU.Time(1:end-1),roll)
nexttile
plot(turn.IMU.Time(1:end-1),pitch)
nexttile
plot(turn.IMU.Time(1:end-1),yaw)
