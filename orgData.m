function [IMU,GPS,REF] = orgData(imu,gps,tru)
%% Organizing IMU data
IMU.t = imu(7,:);
IMU.fx = imu(1,:);
IMU.fy = imu(2,:);
IMU.fz = imu(3,:);
IMU.wx = imu(4,:);
IMU.wy = imu(5,:);
IMU.wz = imu(6,:);

%% Organizing GPS data
GPS.t = gps(7,:);
GPS.x = gps(1,:);
GPS.y = gps(2,:);
GPS.z = gps(3,:);
GPS.x_dot = gps(4,:);
GPS.y_dot = gps(5,:);
GPS.z_dot = gps(6,:);

%% Organizing Ref data
REF.t = tru(16,:);
REF.x = tru(1,:);
REF.y = tru(2,:);
REF.z = tru(3,:);
REF.x_dot = tru(4,:);
REF.y_dot = tru(5,:);
REF.z_dot = tru(6,:);
REF.roll = tru(7,:);
REF.pitch = tru(8,:);
REF.yaw = tru(9,:);
REF.ax = tru(10,:);
REF.ay = tru(11,:);
REF.az = tru(12,:);
REF.wx = tru(13,:);
REF.wy = tru(14,:);
REF.wz = tru(15,:);

% Plotting Raw IMU, Raw GPS, and Reference Solution
IMUfig_acc = figure('Units','normalized','Position',[0.4 0.4 0.6 0.5]);
tiledlayout(3,1)
nexttile
hold on
plot(IMU.t,IMU.fx,'LineWidth',2)
title('Simulated Accelerometers')
ylabel('f^b_{ib_x}[m/s^2]')
xlim([0 IMU.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(IMU.t,IMU.fy,'LineWidth',2)
ylabel('f^b_{ib_y}[m/s^2]')
xlim([0 IMU.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(IMU.t,IMU.fz,'LineWidth',2)
xlabel('Time [s]')
ylabel('f^b_{ib_z}[m/s^2]')
xlim([0 IMU.t(end)])
ax = gca;
ax.FontSize = 18;

IMUfig_ang = figure('Units','normalized','Position',[0.4 0.4 0.6 0.5]);
tiledlayout(3,1)
nexttile
hold on
plot(IMU.t,IMU.wx,'LineWidth',2)
title('Simulated Gyroscopes')
ylabel('\omega^b_{ib_x}[m/s^2]')
xlim([0 IMU.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(IMU.t,IMU.wy,'LineWidth',2)
ylabel('\omega^b_{ib_y}[m/s^2]')
xlim([0 IMU.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(IMU.t,IMU.wz,'LineWidth',2)
xlabel('Time [s]')
ylabel('\omega^b_{ib_z}[m/s^2]')
xlim([0 IMU.t(end)])
ax = gca;
ax.FontSize = 18;

GPSfig_pos = figure('Units','normalized','Position',[0.4 0.4 0.6 0.5]);
tiledlayout(3,1)
nexttile
hold on
plot(GPS.t,GPS.x,'LineWidth',2)
title('Simulated GPS Positioning')
ylabel('x [m]')
xlim([0 GPS.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(GPS.t,GPS.y,'LineWidth',2)
ylabel('y [m]')
xlim([0 GPS.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(GPS.t,GPS.z,'LineWidth',2)
xlabel('Time [s]')
ylabel('z [m]')
xlim([0 GPS.t(end)])
ax = gca;
ax.FontSize = 18;

GPSfig_vel = figure('Units','normalized','Position',[0.4 0.4 0.6 0.5]);
tiledlayout(3,1)
nexttile
hold on
plot(GPS.t,GPS.x_dot,'LineWidth',2)
title('Simulated GPS Positioning')
ylabel('v_x [m/s]')
xlim([0 GPS.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(GPS.t,GPS.y_dot,'LineWidth',2)
ylabel('v_y [m/s]')
xlim([0 GPS.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(GPS.t,GPS.z_dot,'LineWidth',2)
xlabel('Time [s]')
ylabel('v_z [m/s]')
xlim([0 GPS.t(end)])
ax = gca;
ax.FontSize = 18;
end

