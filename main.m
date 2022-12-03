%% Formatting
clc
clear
close all
format shortg

%% Loading in simulated data
load("imu1.mat")
load("ref")
load("gnss.mat")

[IMU,GPS,REF] = orgData(imu1,gnss,ref);

[est] = LC(IMU,GPS,REF);



%% Plotting Raw IMU, Raw GPS, and Reference Solution



% Euler Angles
fig_eul = figure('Units','normalized','Position',[0.4 0.4 0.6 0.5]);
tiledlayout(3,1)
nexttile
hold on
plot(IMU.t,(est.eul(1,:)),'Color','#CC5500','LineWidth',2)
plot(REF.t,REF.roll,'Color','k','LineWidth',2,'LineStyle','--')
title('Dead Reckoned Euler Angles')
ylabel('Roll [deg]')
xlim([0 IMU.t(end)])
legend('Mechanized IMU','Truth','Location','eastoutside')
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(IMU.t,(est.eul(2,:)),'Color','#CC5500','LineWidth',2)
plot(REF.t,REF.pitch,'Color','k','LineWidth',2,'LineStyle','--')
ylabel('Pitch [deg]')
xlim([0 GPS.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(IMU.t,(est.eul(3,:)),'Color','#CC5500','LineWidth',2)
plot(REF.t,REF.yaw,'Color','k','LineWidth',2,'LineStyle','--')
xlabel('Time [s]')
ylabel('Yaw [deg]')
xlim([0 GPS.t(end)])
ax = gca;
ax.FontSize = 18;

% LLA
fig_LLA = figure('Units','normalized','Position',[0.4 0.4 0.6 0.5]);
nexttile
geoplot(rad2deg(est.pos(1,:)),rad2deg(est.pos(2,:)),'Color','#CC5500','LineWidth',2)
hold on
geoplot(REF.LLA(:,1),REF.LLA(:,2),'Color','k','LineWidth',2,'LineStyle','--')
geobasemap satellite
title('Estimated vs. True Position')
ax = gca;
ax.FontSize = 18;

% Estimated Velocity
fig_vel = figure('Units','normalized','Position',[0.4 0.4 0.6 0.5]);
tiledlayout(3,1)
nexttile
hold on
plot(IMU.t,(est.vel(1,:)),'Color','#CC5500','LineWidth',2)
plot(tru_ned(16,:),tru_ned(4,:),'Color','k','LineWidth',2,'LineStyle','--')
title('Estimated Velocity')
ylabel('North [m/s]')
xlim([0 IMU.t(end)])
legend('Mechanized IMU','Truth','Location','eastoutside')
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(IMU.t,(est.vel(2,:)),'Color','#CC5500','LineWidth',2)
plot(tru_ned(16,:),tru_ned(5,:),'Color','k','LineWidth',2,'LineStyle','--')
xlim([0 GPS.t(end)])
ylabel('East [m/s]')
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(IMU.t,(est.vel(3,:)),'Color','#CC5500','LineWidth',2)
plot(tru_ned(16,:),tru_ned(6,:),'Color','k','LineWidth',2,'LineStyle','--')
ylabel('Down [m/s]')
xlim([0 GPS.t(end)])
ax = gca;
ax.FontSize = 18;

figure
plot3(tru_ned(1,:),tru_ned(2,:),tru_ned(3,:))
