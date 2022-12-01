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
LC_random = gpsimuLC(static_data,random_data);

%% Plotting
% Starting with euler angles
eul_turn_LC = figure('Units','normalized','Position',[0.3 0.3 0.6 0.5]);
eul_turn_LC_tl = tiledlayout(3,1,"TileSpacing","compact","Padding","compact");
nexttile
hold on
plot(turn_data.memsense.imuTimeReference.time,movmean((rad2deg(LC_turn.roll)),25),'LineWidth',1.25)
ylim([-125 125])
yticks([-90 0 90])
title('Estimated Euler Angles: Turn Dataset')
ylabel('Roll [deg]')
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(turn_data.memsense.imuTimeReference.time,movmean((rad2deg(LC_turn.pitch)),10),'LineWidth',1.25)
ylim([-45 45])
ylabel('Pitch [deg]')
yticks([-45 0 45])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(turn_data.memsense.imuTimeReference.time,movmean((rad2deg(LC_turn.yaw)),50),'LineWidth',1.25)
ylim([-180 180])
yticks([-180 -90 0 90 180])
ylabel('Yaw [deg]')
xlabel('Time [s]')
ax = gca;
ax.FontSize = 18;

eul_random_LC = figure('Units','normalized','Position',[0.3 0.3 0.6 0.5]);
eul_random_LC_tl = tiledlayout(3,1,"TileSpacing","compact","Padding","compact");
nexttile
hold on
plot(random_data.memsense.imuTimeReference.time,wrapTo180(rad2deg(LC_random.roll)-135),'LineWidth',1.25)
ylim([-125 125])
yticks([-90 0 90])
title('Estimated Euler Angles: Random Dataset')
ylabel('Roll [deg]')
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(random_data.memsense.imuTimeReference.time,wrapTo180(rad2deg(LC_random.pitch)),'LineWidth',1.25)
ylim([-45 45])
ylabel('Pitch [deg]')
yticks([-45 0 45])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(random_data.memsense.imuTimeReference.time,wrapTo180(rad2deg(LC_random.yaw)-90),'LineWidth',1.25)
ylim([-180 180])
yticks([-180 -90 0 90 180])
ylabel('Yaw [deg]')
xlabel('Time [s]')
ax = gca;
ax.FontSize = 18;
% 
% eul_static_LC = figure('Units','normalized','Position',[0.3 0.3 0.6 0.5]);
% eul_static_LC_tl = tiledlayout(3,1,"TileSpacing","compact","Padding","compact");
% nexttile
% hold on
% plot(static_data.memsense.imuTimeReference.time,wrapTo180(rad2deg(LC_static.roll)-135),'LineWidth',1.25)
% ylim([-125 125])
% yticks([-90 0 90])
% title('Estimated Euler Angles: Random Dataset')
% ylabel('Roll [deg]')
% ax = gca;
% ax.FontSize = 18;
% nexttile
% hold on
% plot(static_data.memsense.imuTimeReference.time,wrapTo180(rad2deg(LC_static.pitch)),'LineWidth',1.25)
% ylim([-45 45])
% ylabel('Pitch [deg]')
% yticks([-45 0 45])
% ax = gca;
% ax.FontSize = 18;
% nexttile
% hold on
% plot(static_data.memsense.imuTimeReference.time,wrapTo180(rad2deg(LC_static.yaw)-90),'LineWidth',1.25)
% ylim([-180 180])
% yticks([-180 -90 0 90 180])
% ylabel('Yaw [deg]')
% xlabel('Time [s]')
% ax = gca;
% ax.FontSize = 18;

% Next plot LLA
lla_turn_LC = figure('Units','normalized','Position',[0.3 0.3 0.6 0.5]);
estiLat = rad2deg(LC_turn.lat);
estiLong = rad2deg(LC_turn.lon);
truthLat = (turn_data.novatel_local.llhPositionTagged.latitude);
truthLong = (turn_data.novatel_local.llhPositionTagged.longitude);
geoplot(estiLat,estiLong,'LineWidth',2)
hold on
geoplot(truthLat,truthLong,'LineWidth',2)
geobasemap satellite
title('Turn Dataset')
ax = gca;
ax.FontSize = 18;

% lla_random_LC = figure('Units','normalized','Position',[0.3 0.3 0.6 0.5]);
% geoplot(rad2deg(LC_random.lat),rad2deg(LC_random.lon),'LineWidth',2)
% geobasemap satellite
% title('Random Dataset')
% ax = gca;
% ax.FontSize = 18;
% 
% lla_static_LC = figure('Units','normalized','Position',[0.3 0.3 0.6 0.5]);
% geoplot(rad2deg(LC_static.lat),rad2deg(LC_static.lon),'LineWidth',2)
% geobasemap satellite
% title('Static Dataset')
% ax = gca;
% ax.FontSize = 18;

lla_random_gps = figure('Units','normalized','Position',[0.3 0.3 0.6 0.5]);
geoplot((random_data.novatel_local.llhPositionTagged.latitude),(random_data.novatel_local.llhPositionTagged.longitude),'LineWidth',2)
geobasemap satellite
title('Random Dataset')
ax = gca;
ax.FontSize = 18;