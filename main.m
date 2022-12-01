%% Formatting
clc
clear
close all
format shortg
addpath("data\")
%% Loading in .mat files from Truck data
load("random_raw.mat","random_data")
load("static_raw.mat","static_data")
load("turn_raw.mat","turn_data")

LC_turn = gpsimuLC(static_data,turn_data);
LC_static = gpsimuLC(static_data,static_data);
LC_random = gpsimuLC(static_data,random_data);

%% Plotting
% Starting with euler angles
eul_turn_LC = figure('Units','normalized','Position',[0.3 0.3 0.6 0.5]);
eul_turn_LC_tl = tiledlayout(3,1,"TileSpacing","compact","Padding","compact");
nexttile
hold on
plot(turn_data.memsense.imuTimeReference.time,wrapTo180(rad2deg(LC_turn.roll)))
ylim([-180 180])
title('Estimated Euler Angles: Turn Dataset')
ylabel('Roll [deg]')
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(turn_data.memsense.imuTimeReference.time,wrapTo180(rad2deg(LC_turn.pitch)))
ylim([-90 90])
ylabel('Pitch [deg]')
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(turn_data.memsense.imuTimeReference.time,wrapTo180(rad2deg(LC_turn.yaw)))
ylabel('Yaw [deg]')
xlabel('Time [s]')
ax = gca;
ax.FontSize = 18;