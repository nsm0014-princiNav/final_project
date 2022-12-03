function [IMU,GPS,REF] = orgData(imu,gps,ref)

% Raw Accelerometers
IMUfig_acc = figure('Units','normalized','Position',[0.4 0.4 0.6 0.5]);
tiledlayout(3,1)
nexttile
hold on
plot(imu.t,imu.fb(:,1),'LineWidth',2)
title('Simulated Accelerometers')
ylabel('f^b_{ib_x}[m/s^2]')
xlim([0 imu.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(imu.t,imu.fb(:,2),'LineWidth',2)
ylabel('f^b_{ib_y}[m/s^2]')
xlim([0 imu.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(imu.t,imu.fb(:,3),'LineWidth',2)
xlabel('Time [s]')
ylabel('f^b_{ib_z}[m/s^2]')
xlim([0 imu.t(end)])
ax = gca;
ax.FontSize = 18;

% Raw Gyroscopes
IMUfig_ang = figure('Units','normalized','Position',[0.4 0.4 0.6 0.5]);
tiledlayout(3,1)
nexttile
hold on
plot(imu.t,imu.wb(:,1),'LineWidth',2)
title('Simulated Gyroscopes')
ylabel('\omega^b_{ib_x}[rad/s]')
xlim([0 imu.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(imu.t,imu.wb(:,2),'LineWidth',2)
ylabel('\omega^b_{ib_y}[rad/s]')
xlim([0 imu.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(imu.t,imu.wb(:,3),'LineWidth',2)
xlabel('Time [s]')
ylabel('\omega^b_{ib_z}[rad/s]')
xlim([0 imu.t(end)])
ax = gca;
ax.FontSize = 18;

% position
GPSfig_pos = figure('Units','normalized','Position',[0.4 0.4 0.6 0.5]);
tiledlayout(3,1)
nexttile
hold on
plot(GPS.t,GPS.x,'LineWidth',2)
plot(REF.t,REF.x,'Color','k','LineWidth',2,'LineStyle','--')
title('GPS Position')
ylabel('x [m]')
xlim([0 GPS.t(end)])
legend('Simulated GPS','Truth','Location','eastoutside')
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(GPS.t,GPS.y,'LineWidth',2)
plot(REF.t,REF.y,'Color','k','LineWidth',2,'LineStyle','--')
ylabel('y [m]')
xlim([0 GPS.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(GPS.t,GPS.z,'LineWidth',2)
plot(REF.t,REF.z,'Color','k','LineWidth',2,'LineStyle','--')
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
title('GPS Velocity')
ylabel('v_x [m/s]')
xlim([0 GPS.t(end)])
legend('Simulated GPS','Truth','Location','eastoutside')
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(GPS.t,GPS.y_dot,'LineWidth',2)
plot(REF.t,REF.y_dot,'Color','k','LineWidth',2,'LineStyle','--')
ylabel('v_y [m/s]')
xlim([0 GPS.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(GPS.t,GPS.z_dot,'LineWidth',2)
plot(REF.t,REF.z_dot,'Color','k','LineWidth',2,'LineStyle','--')
xlabel('Time [s]')
ylabel('v_z [m/s]')
xlim([0 GPS.t(end)])
ax = gca;
ax.FontSize = 18;

end

