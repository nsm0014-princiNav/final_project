%% Formatting
clc
clear
close all
format shortg

%% Loading in simulated data
load("sim_lap_3D.mat")

% Defining Noise Characteristics for IMU - Tactical Grade pg.153
noise.acc.drift = ([randn(1) randn(1) randn(1)].*0.5); % [m/s^2]
noise.acc.bias = ([randn(1) randn(1) randn(1)].*0.5); % [m/s^2]
noise.gyr.drift = ([randn(1) randn(1) randn(1)].*0.5); % [rad/s]
noise.gyr.bias = ([randn(1) randn(1) randn(1)].*0.005); % [rad/s]
noise.gps_pos.drift = [randn(1)*3 randn(1)*3 randn(1)*5];
noise.gps_vel.drift = [randn(1)*.3 randn(1)*.3 randn(1)*.5];

% noise.acc.drift = ([randn(1) randn(1) randn(1)].*0.0); % [m/s^2]
% noise.acc.bias = ([randn(1) randn(1) randn(1)].*0.0); % [m/s^2]
% noise.gyr.drift = ([randn(1) randn(1) randn(1)].*0.0); % [rad/s]
% noise.gyr.bias = ([randn(1) randn(1) randn(1)].*0.000); % [rad/s]
% noise.gps_pos.drift = [randn(1)*0 randn(1)*0 randn(1)*0];
% noise.gps_vel.drift = [randn(1)*0 randn(1)*0 randn(1)*0];

[IMU,GPS,REF] = orgData(imu,gps,tru,noise,ref_lla);

[est] = LC(IMU,GPS,REF);



%% Plotting Raw IMU, Raw GPS, and Reference Solution

% Raw Accelerometers
IMUfig_acc = figure('Units','normalized','Position',[0.4 0.4 0.6 0.5]);
tiledlayout(3,1)
nexttile
hold on
plot(IMU.t,IMU.fx,'LineWidth',2)
plot(REF.t,REF.ax,'Color','k','LineWidth',2,'LineStyle','--')
title('Simulated Accelerometers')
ylabel('f^b_{ib_x}[m/s^2]')
xlim([0 IMU.t(end)])
legend('Simulated','Truth','Location','eastoutside')
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(IMU.t,IMU.fy,'LineWidth',2)
plot(REF.t,REF.ay,'Color','k','LineWidth',2,'LineStyle','--')
ylabel('f^b_{ib_y}[m/s^2]')
xlim([0 IMU.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(IMU.t,IMU.fz,'LineWidth',2)
plot(REF.t,REF.az,'Color','k','LineWidth',2,'LineStyle','--')
xlabel('Time [s]')
ylabel('f^b_{ib_z}[m/s^2]')
xlim([0 IMU.t(end)])
ax = gca;
ax.FontSize = 18;

% Raw Gyroscopes
IMUfig_ang = figure('Units','normalized','Position',[0.4 0.4 0.6 0.5]);
tiledlayout(3,1)
nexttile
hold on
plot(IMU.t,IMU.wx,'LineWidth',2)
plot(REF.t,REF.wx,'Color','k','LineWidth',2,'LineStyle','--')
legend('Simulated','Truth','Location','eastoutside')
title('Simulated Gyroscopes')
ylabel('\omega^b_{ib_x}[m/s^2]')
xlim([0 IMU.t(end)])
ylim([min(IMU.wx)-0.1 max(IMU.wx)+0.1])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(IMU.t,IMU.wy,'LineWidth',2)
plot(REF.t,REF.wy,'Color','k','LineWidth',2,'LineStyle','--')
ylabel('\omega^b_{ib_y}[m/s^2]')
xlim([0 IMU.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(IMU.t,IMU.wz,'LineWidth',2)
plot(REF.t,REF.wz,'Color','k','LineWidth',2,'LineStyle','--')
xlabel('Time [s]')
ylabel('\omega^b_{ib_z}[m/s^2]')
xlim([0 IMU.t(end)])
ax = gca;
ax.FontSize = 18;

% position
GPSfig_pos = figure('Units','normalized','Position',[0.4 0.4 0.6 0.5]);
tiledlayout(3,1)
nexttile
hold on
plot(GPS.t,GPS.x,'LineWidth',2)
plot(REF.t,REF.x,'Color','k','LineWidth',2,'LineStyle','--')
plot(IMU.t,est.pos(1,:),'Color','#CC5500','LineWidth',2)
title('Dead Reckoned Position')
ylabel('x [m]')
xlim([0 GPS.t(end)])
legend('Simulated GPS','Truth','Estimate','Location','eastoutside')
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(GPS.t,GPS.y,'LineWidth',2)
plot(REF.t,REF.y,'Color','k','LineWidth',2,'LineStyle','--')
plot(IMU.t,est.pos(2,:),'Color','#CC5500','LineWidth',2)
ylabel('y [m]')
xlim([0 GPS.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(GPS.t,GPS.z,'LineWidth',2)
plot(REF.t,REF.z,'Color','k','LineWidth',2,'LineStyle','--')
plot(IMU.t,est.pos(3,:),'Color','#CC5500','LineWidth',2)
xlabel('Time [s]')
ylabel('z [m]')
xlim([0 GPS.t(end)])
ax = gca;
ax.FontSize = 18;
% Velocity
GPSfig_vel = figure('Units','normalized','Position',[0.4 0.4 0.6 0.5]);
tiledlayout(3,1)
nexttile
hold on
plot(GPS.t,GPS.x_dot,'LineWidth',2)
plot(REF.t,REF.x_dot,'Color','k','LineWidth',2,'LineStyle','--')
plot(IMU.t,est.vel(1,:),'Color','#CC5500','LineWidth',2)
title('Dead Reckoned Velocity')
ylabel('v_x [m/s]')
xlim([0 GPS.t(end)])
legend('Simulated GPS','Truth','Estimate','Location','eastoutside')
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(GPS.t,GPS.y_dot,'LineWidth',2)
plot(REF.t,REF.y_dot,'Color','k','LineWidth',2,'LineStyle','--')
plot(IMU.t,est.vel(2,:),'Color','#CC5500','LineWidth',2)
ylabel('v_y [m/s]')
xlim([0 GPS.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(GPS.t,GPS.z_dot,'LineWidth',2)
plot(REF.t,REF.z_dot,'Color','k','LineWidth',2,'LineStyle','--')
plot(IMU.t,est.vel(3,:),'Color','#CC5500','LineWidth',2)
xlabel('Time [s]')
ylabel('v_z [m/s]')
xlim([0 GPS.t(end)])
ax = gca;
ax.FontSize = 18;

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
geoplot(est.pos_lla(1,:),est.pos_lla(2,:),'Color','#CC5500','LineWidth',2)
hold on
geoplot(REF.LLA(:,1),REF.LLA(:,2),'Color','k','LineWidth',2,'LineStyle','--')
geobasemap satellite
title('Estimated vs. True Position')
ax = gca;
ax.FontSize = 18;
