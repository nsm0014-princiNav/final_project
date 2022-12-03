function plotData(esti,ref)
% plot size
plotSize = [0.25 0.25 0.3 0.5];
weight = 2;
R2D = 180/pi;
%% 3D TRAJECTORY
fig_3D = figure('Units','normalized','Position',[0.25 0.25 0.5 0.5],'Name','3D Trajectory');
hold on
plot3(esti.lon.*R2D, esti.lat.*R2D, esti.h,'LineWidth', weight)
plot3(ref.lon.*R2D, ref.lat.*R2D, ref.h, '--k','LineWidth', weight)
axis tight
title('3D TRAJECTORY')
xlabel('Longitude [deg]','Interpreter','latex')
ylabel('Latitude [deg]','Interpreter','latex')
zlabel('h [m]','Interpreter','latex')
view(-45,25)
legend('Estimate','Reference','Location','eastoutside')
ax = gca;
ax.FontSize = 18;
grid

%% ATTITUDE
fig_eul = figure('Units','normalized','Position',plotSize,'Name','Euler Angles');
tiledlayout(3,1)
nexttile
hold on
plot(esti.t, R2D.*esti.roll,'LineWidth', weight)
plot(ref.t, R2D.*ref.roll, '--k','LineWidth', weight)
xlim([esti.tg(1) esti.tg(end)])
ylabel('$$\phi$$ [deg]','Interpreter','latex')
legend('Estimate','Reference','Location','eastoutside')
title('Estimated Euler Angles')
ax = gca;
ax.FontSize = 18;
grid

nexttile
hold on
plot(esti.t, R2D.*esti.pitch,'LineWidth', weight)
plot(ref.t, R2D.*ref.pitch, '--k','LineWidth', weight)
xlim([esti.tg(1) esti.tg(end)])
ylabel('$$\theta$$ [deg]','Interpreter','latex')
ax = gca;
ax.FontSize = 18;
grid

nexttile
hold on
plot(esti.t, R2D.*esti.yaw,'LineWidth', weight)
plot(ref.t, R2D.* ref.yaw, '--k','LineWidth', weight)
xlim([esti.tg(1) esti.tg(end)])
ylabel('$$\psi$$ [deg]','Interpreter','latex')
xlabel('Time [s]')
ax = gca;
ax.FontSize = 18;
grid

%% ATTITUDE ERRORS
fig_eul_err = figure('Units','normalized','Position',plotSize,'Name','Euler Angle Errors');
tiledlayout(3,1)
nexttile
hold on
plot(esti.t, R2D.*(esti.roll - ref.roll),'LineWidth', weight)
xlim([esti.tg(1) esti.tg(end)])
ylabel('$$\Delta\phi$$ [deg]','Interpreter','latex')
title('Euler Angle Errors')
ax = gca;
ax.FontSize = 18;
grid

nexttile
hold on
plot(esti.t, R2D.*(esti.pitch - ref.pitch),'LineWidth', weight)
xlim([esti.tg(1) esti.tg(end)])
ylabel('$$\Delta\theta$$ [deg]','Interpreter','latex')
ax = gca;
ax.FontSize = 18;
grid

nexttile
hold on
plot(esti.t, R2D.*(esti.yaw - ref.yaw),'LineWidth', weight)
xlim([esti.tg(1) esti.tg(end)])
ylabel('$$\Delta\psi$$ [deg]','Interpreter','latex')
xlabel('Time [s]')
ax = gca;
ax.FontSize = 18;
grid
%% VELOCITY ESTIMATE
fig_vel = figure('Units','normalized','Position',plotSize,'Name','NED Velocities');
tiledlayout(3,1)
nexttile
hold on
plot(esti.t, esti.vel(:,1),'LineWidth', 2)
plot(ref.t, ref.vel(:,1), '--k','LineWidth', 2)
ylabel('V_N [m/s]')
title('Estimated Velocities: NED')
legend('Estimate','Reference','Location','eastoutside')
xlim([esti.tg(1) esti.tg(end)])
ax = gca;
ax.FontSize = 18;
grid

nexttile
hold on
plot(esti.t, esti.vel(:,2),'LineWidth', 2)
plot(ref.t, ref.vel(:,2), '--k','LineWidth', 2)
xlim([esti.tg(1) esti.tg(end)])
ylabel('V_E [m/s]')
ax = gca;
ax.FontSize = 18;
grid

nexttile
hold on
plot(esti.t, esti.vel(:,3),'LineWidth', 2)
plot(ref.t, ref.vel(:,3), '--k','LineWidth', 2)
xlim([esti.tg(1) esti.tg(end)])
xlabel('Time [s]')
ylabel('V_D [m/s]')
ax = gca;
ax.FontSize = 18;
grid

%% VELOCITY ERRORS
fig_vel_err = figure('Units','normalized','Position',plotSize,'Name','NED Velocity Errors');
tiledlayout(3,1)
nexttile
hold on
plot(esti.t, esti.vel(:,1) - ref.vel(:,1),'LineWidth', 2)
ylabel('$$\Delta V_N$$ [m/s]','Interpreter','latex')
title('Velocity Errors')
xlim([esti.tg(1) esti.tg(end)])
ax = gca;
ax.FontSize = 18;
grid

nexttile
hold on
plot(esti.t, esti.vel(:,2) - ref.vel(:,2),'LineWidth', 2)
ylabel('$$\Delta V_E$$ [m/s]','Interpreter','latex')
xlim([esti.tg(1) esti.tg(end)])
ax = gca;
ax.FontSize = 18;
grid

nexttile
hold on
plot(esti.t, esti.vel(:,3) - ref.vel(:,3),'LineWidth', 2)
ylabel('$$\Delta V_D$$ [m/s]','Interpreter','latex')
xlim([esti.tg(1) esti.tg(end)])
xlabel('Time [s]')
ax = gca;
ax.FontSize = 18;
grid
%% POSITION ESTIMATE
fig_pos = figure('Units','normalized','Position',plotSize,'Name','Positions: LLA');
tiledlayout(3,1)
nexttile
hold on
plot(esti.t, esti.lat.*R2D,'LineWidth', 2)
plot(ref.t, ref.lat .*R2D, '--k','LineWidth', 2)
xlim([esti.tg(1) esti.tg(end)])
ylabel('$$\hat{\mathcal{L}}$$ [deg]','Interpreter','latex')
title('Estimated Position: LLA')
legend('Estimate','Reference','Location','eastoutside')
ax = gca;
ax.FontSize = 18;
grid

nexttile
hold on
plot(esti.t, esti.lon.*R2D,'LineWidth', 2)
plot(ref.t, ref.lon .*R2D, '--k','LineWidth', 2)
xlim([esti.tg(1) esti.tg(end)])
ylabel('$$\hat{\lambda}$$ [deg]','Interpreter','latex')
ax = gca;
ax.FontSize = 18;
grid

nexttile
hold on
plot(esti.t, esti.h,'LineWidth', 2)
plot(ref.t, ref.h, '--k','LineWidth', 2)
xlim([esti.tg(1) esti.tg(end)])
xlabel('Time [s]')
ylabel('h [m]','Interpreter','latex')
ax = gca;
ax.FontSize = 18;
grid

%% POSITION ERRORS
fig_pos_err = figure('Units','normalized','Position',plotSize,'Name','Position Errors');
tiledlayout(3,1)
nexttile
hold on
plot(esti.t, (esti.lat - ref.lat).*R2D,'LineWidth', 2)
xlim([esti.tg(1) esti.tg(end)])
ylabel('$$\Delta{\mathcal{L}}$$ [deg]','Interpreter','latex')
title('Position Errors')
ax = gca;
ax.FontSize = 18;
grid

nexttile
hold on
plot(esti.t, (esti.lon - ref.lon).*R2D,'LineWidth', 2)
xlim([esti.tg(1) esti.tg(end)])
ylabel('$$\Delta{\lambda}$$ [deg]','Interpreter','latex')
ax = gca;
ax.FontSize = 18;
grid

nexttile
hold on
plot(esti.t, (esti.h - ref.h).*R2D,'LineWidth', 2)
xlim([esti.tg(1) esti.tg(end)])
xlabel('Time [s]')
ylabel('$$\Delta h$$ [m]','Interpreter','latex')
ax = gca;
ax.FontSize = 18;
grid

%% BIAS ESTIMATION
fig_gyr_bias = figure('Units','normalized','Position',plotSize,'Name','Gyroscope Biases');
tiledlayout(3,1)
nexttile
plot(esti.tg, esti.b(:, 1).*R2D,'LineWidth',weight)
xlim([esti.tg(1) esti.tg(end)])
ylabel('$$\hat{b}_{g_r}$$ [deg]','Interpreter','Latex')
title('Estimated Gyroscope Biases')
ax = gca;
ax.FontSize = 18;
grid

nexttile
plot(esti.tg, esti.b(:, 2).*R2D,'LineWidth',weight)
xlim([esti.tg(1) esti.tg(end)])
ylabel('$$\hat{b}_{g_p}$$ [deg]','Interpreter','Latex')
ax = gca;
ax.FontSize = 18;
grid

nexttile
plot(esti.tg, esti.b(:, 3).*R2D,'LineWidth',weight)
xlim([esti.tg(1) esti.tg(end)])
xlabel('Time [s]')
ylabel('$$\hat{b}_{g_y}$$ [deg]','Interpreter','Latex')
ax = gca;
ax.FontSize = 18;
grid

fig_acc_bias = figure('Units','normalized','Position',plotSize,'Name','Accelerometer Biases');
tiledlayout(3,1)
nexttile
plot(esti.tg, esti.b(:, 4),'LineWidth',weight)
xlim([esti.tg(1) esti.tg(end)])
ylabel('$$\hat{b}_{a_x}$$ [m/$$s^2$$]','Interpreter','Latex')
title('Estimated Accelerometer Biases')
ax = gca;
ax.FontSize = 18;
grid

nexttile
plot(esti.tg, esti.b(:, 5),'LineWidth',weight)
xlim([esti.tg(1) esti.tg(end)])
ylabel('$$\hat{b}_{a_y}$$ [m/$$s^2$$]','Interpreter','Latex')
ax = gca;
ax.FontSize = 18;
grid

nexttile
plot(esti.tg, esti.b(:, 6),'LineWidth',weight)
xlim([esti.tg(1) esti.tg(end)])
xlabel('Time [s]')
ylabel('$$\hat{b}_{a_z}$$ [m/$$s^2$$]','Interpreter','Latex')
ax = gca;
ax.FontSize = 18;
grid
end