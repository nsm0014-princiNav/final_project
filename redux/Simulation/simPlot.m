function simPlot(ref,imu,gnss)

%% Raw Accelerometers
IMUfig_acc = figure('Units','normalized','Position',[0.4 0.4 0.45 0.5]);
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
saveas(IMUfig_acc,"figures\IMU_ACC.png")

%% Raw Gyroscopes
IMUfig_ang = figure('Units','normalized','Position',[0.4 0.4 0.45 0.5]);
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
saveas(IMUfig_ang,"figures\IMU_GYR.png")

%% GPS Position
GPSfig_pos = figure('Units','normalized','Position',[0.4 0.4 0.45 0.5]);
hold on
plot3(gnss.lat,gnss.lon,gnss.h)
plot3(ref.lat,ref.lon,ref.h,'Color','k','LineWidth',2,'LineStyle','--')
view(gca,150,22)
title('Simulated GPS Position')
zlabel('Height [m]')
ylabel('Longitude [deg]')
xlabel('Latitude [deg]')
ax = gca;
ax.FontSize = 18;
saveas(GPSfig_pos,"figures\GPS_POSITION.png")

%% GPS Velocity
GPSfig_vel = figure('Units','normalized','Position',[0.4 0.4 0.45 0.5]);
tiledlayout(3,1)
nexttile
hold on
plot(gnss.t,gnss.vel(:,1),'LineWidth',2)
title('Simulated GPS Velocity')
ylabel('North [m/s]')
xlim([0 imu.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(gnss.t,gnss.vel(:,2),'LineWidth',2)
ylabel('East [m/s]')
xlim([0 imu.t(end)])
ax = gca;
ax.FontSize = 18;
nexttile
hold on
plot(gnss.t,gnss.vel(:,3),'LineWidth',2)
xlabel('Time [s]')
ylabel('Down [m/s]')
xlim([0 imu.t(end)])
ax = gca;
ax.FontSize = 18;
saveas(GPSfig_vel,"figures\GPS_VELOCITY.png")

end