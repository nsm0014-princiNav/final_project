%% Formatting
clc
clear
close all
format shortg

%% CONVERSION CONSTANTS

G =  9.80665;       % Gravity constant, m/s^2
G2MSS = G;          % g to m/s^2
MSS2G = (1/G);      % m/s^2 to g

D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees

KT2MS = 0.514444;   % knot to m/s
MS2KMH = 3.6;       % m/s to km/h

%% REFERENCE DATA

fprintf('NaveGo: loading reference dataset from a trajectory generator... \n')
load sim_lap_3D.mat
load ref.mat

lla = ecef2lla(tru(1:3,:)');
ned = ecef2ned(tru(1:3,:)',deg2rad(lla(1,1:3)));
nedv = [0 0 0;diff(ned)].*100;
for i = 1:length(tru(16,:))
DCMnb_m(:,:,i) = euler2dcm([deg2rad(tru(13,i))' deg2rad(tru(14,i))' deg2rad(tru(15,i))']);
temp(i,:) = [DCMnb_m(1:3,1,i)' DCMnb_m(1:3,2,i)' DCMnb_m(1:3,3,i)'];
end

stop = 1;
ref = struct( ...
    't',tru(16,:)',...
    'lat',deg2rad(lla(:,1)),...
    'lon',deg2rad(lla(:,2)),...
    'h',lla(:,3),...
    'vel',nedv,...
    'roll',deg2rad(tru(13,:))',...
    'pitch',deg2rad(tru(14,:))',...
    'yaw' ,deg2rad(tru(15,:))',...
    'DCMnb_m',temp, ...
    'freq',100 ...
    );
save ../ref.mat ref
% ref.mat contains the reference data structure from which inertial
% sensors and GNSS wil be simulated. It must contain the following fields:

%         t: Nx1 time vector (seconds).
%       lat: Nx1 latitude (radians).
%       lon: Nx1 longitude (radians).
%         h: Nx1 altitude (m).
%       vel: Nx3 NED velocities (m/s).
%      roll: Nx1 roll angles (radians).
%     pitch: Nx1 pitch angles (radians).
%       yaw: Nx1 yaw angle vector (radians).
%   DCMnb_m: Nx9 matrix with nav-to-body direct cosine matrices (DCM).
%            Each row of DCMnb_m contains the 9 elements of a particular DCMnb
%            matrix ordered as [a11 a21 a31 a12 a22 a32 a13 a23 a33].
%      freq: sampling frequency (Hz).

%% ADIS16405 IMU ERROR PROFILE

% IMU data structure:
%         t: Ix1 time vector (seconds).
%        fb: Ix3 accelerations vector in body frame XYZ (m/s^2).
%        wb: Ix3 turn rates vector in body frame XYZ (radians/s).
%       arw: 1x3 angle random walks (rad/s/root-Hz).
%      arrw: 1x3 angle rate random walks (rad/s^2/root-Hz).
%       vrw: 1x3 velocity random walks (m/s^2/root-Hz).
%      vrrw: 1x3 velocity rate random walks (m/s^3/root-Hz).
%     g_std: 1x3 gyros standard deviations (radians/s).
%     a_std: 1x3 accrs standard deviations (m/s^2).
%    gb_sta: 1x3 gyros static biases or turn-on biases (radians/s).
%    ab_sta: 1x3 accrs static biases or turn-on biases (m/s^2).
%    gb_dyn: 1x3 gyros dynamic biases or bias instabilities (radians/s).
%    ab_dyn: 1x3 accrs dynamic biases or bias instabilities (m/s^2).
%   gb_corr: 1x3 gyros correlation times (seconds).
%   ab_corr: 1x3 accrs correlation times (seconds).
%    gb_psd: 1x3 gyros dynamic biases root-PSD (rad/s/root-Hz).
%    ab_psd: 1x3 accrs dynamic biases root-PSD (m/s^2/root-Hz);
%      freq: 1x1 sampling frequency (Hz).
% ini_align: 1x3 initial attitude at t(1), [roll pitch yaw] (rad).
% ini_align_err: 1x3 initial attitude errors at t(1), [roll pitch yaw] (rad).

ADIS16405.arw      = 2   .* ones(1,3);     % Angle random walks [X Y Z] (deg/root-hour)
ADIS16405.arrw     = zeros(1,3);           % Angle rate random walks [X Y Z] (deg/root-hour/s)
ADIS16405.vrw      = 0.2 .* ones(1,3);     % Velocity random walks [X Y Z] (m/s/root-hour)
ADIS16405.vrrw     = zeros(1,3);           % Velocity rate random walks [X Y Z] (deg/root-hour/s)
ADIS16405.gb_sta   = 3   .* ones(1,3);     % Gyro static biases [X Y Z] (deg/s)
ADIS16405.ab_sta   = 50  .* ones(1,3);     % Acc static biases [X Y Z] (mg)
ADIS16405.gb_dyn   = 0.007 .* ones(1,3);   % Gyro dynamic biases [X Y Z] (deg/s)
ADIS16405.ab_dyn   = 0.2 .* ones(1,3);     % Acc dynamic biases [X Y Z] (mg)
ADIS16405.gb_corr  = 100 .* ones(1,3);     % Gyro correlation times [X Y Z] (seconds)
ADIS16405.ab_corr  = 100 .* ones(1,3);     % Acc correlation times [X Y Z] (seconds)
ADIS16405.freq     = ref.freq;             % IMU operation frequency [X Y Z] (Hz)
ADIS16405.m_psd    = 0.066 .* ones(1,3);   % Magnetometer noise density [X Y Z] (mgauss/root-Hz)

% ref time is used to simulate IMU sensors
ADIS16405.t = ref.t;                       % IMU time vector
dt = mean(diff(ADIS16405.t));              % IMU sampling interval

imu1 = imu_si_errors(ADIS16405, dt);       % IMU manufacturer error units to SI units.

imu1.ini_align_err = [1 1 2] .* D2R;                % Initial attitude align errors for matrix P in Kalman filter, [roll pitch yaw] (radians)
imu1.ini_align = [ref.roll(1) ref.pitch(1) ref.yaw(1)]; % Initial attitude align at t(1) (radians).
imu1.t = ADIS16405.t;

%% ADIS16488 IMU ERROR PROFILE

ADIS16488.arw      = 0.3  .* ones(1,3);     % Angle random walks [X Y Z] (deg/root-hour)
ADIS16488.arrw     = zeros(1,3);            % Angle rate random walks [X Y Z] (deg/root-hour/s)
ADIS16488.vrw      = 0.029.* ones(1,3);     % Velocity random walks [X Y Z] (m/s/root-hour)
ADIS16488.vrrw     = zeros(1,3);            % Velocity rate random walks [X Y Z] (deg/root-hour/s)
ADIS16488.gb_sta   = 0.2  .* ones(1,3);     % Gyro static biases [X Y Z] (deg/s)
ADIS16488.ab_sta   = 16   .* ones(1,3);     % Acc static biases [X Y Z] (mg)
ADIS16488.gb_dyn   = 6.5/3600 .* ones(1,3); % Gyro dynamic biases [X Y Z] (deg/s)
ADIS16488.ab_dyn   = 0.1  .* ones(1,3);     % Acc dynamic biases [X Y Z] (mg)
ADIS16488.gb_corr  = 100  .* ones(1,3);     % Gyro correlation times [X Y Z] (seconds)
ADIS16488.ab_corr  = 100  .* ones(1,3);     % Acc correlation times [X Y Z] (seconds)
ADIS16488.freq     = ref.freq;              % IMU operation frequency [X Y Z] (Hz)
ADIS16488.m_psd    = 0.054 .* ones(1,3);    % Magnetometer noise density [X Y Z] (mgauss/root-Hz)

% ref time is used to simulate IMU sensors
ADIS16488.t = ref.t;                        % IMU time vector
dt = mean(diff(ADIS16488.t));               % IMU sampling interval

%% GARMIN 5-18 Hz GPS ERROR PROFILE

% GNSS data structure:
%         t: Mx1 time vector (seconds).
%       lat: Mx1 latitude (radians).
%       lon: Mx1 longitude (radians).
%         h: Mx1 altitude (m).
%       vel: Mx3 NED velocities (m/s).
%       std: 1x3 position standard deviations, [lat lon h] (rad, rad, m).
%      stdm: 1x3 position standard deviations, [lat lon h] (m, m, m).
%      stdv: 1x3 velocity standard deviations, [Vn Ve Vd] (m/s).
%      larm: 3x1 lever arm from IMU to GNSS antenna (x-fwd, y-right, z-down) (m).
%      freq: 1x1 sampling frequency (Hz).
%   zupt_th: 1x1 ZUPT threshold (m/s).
%  zupt_win: 1x1 ZUPT time window (seconds).
%       eps: 1x1 time interval to compare IMU time vector to GNSS time vector (seconds).

gnss.stdm = [5 5 10];                   % GNSS positions standard deviations [lat lon h] (meters)
gnss.stdv = 0.10 * KT2MS .* ones(1,3);  % GNSS velocities standard deviations [Vn Ve Vd] (meters/s)
gnss.larm = zeros(3,1);                 % GNSS lever arm from IMU to GNSS antenna (x-fwd, y-right, z-down) (meters).
gnss.freq = 10;                          % GNSS operation frequency (Hz)

% Parameters for ZUPT detection algorithm
gnss.zupt_th = 0.5;   % ZUPT threshold (m/s).
gnss.zupt_win = 4;    % ZUPT time window (seconds).

% The following figure tries to show when the Kalman filter (KF) will be execute.
% If a new element from GNSS time vector is available at the current INS
% time, within the window time [-eps eps] set by epsilon, the Kalman filter (KF) will be executed.
%
%                    I1  I2  I3  I4  I5  I6  I7  I8  I9  I10 I11 I12 I3
% INS time vector:   |---|---|---|---|---|---|---|---|---|---|---|---|
% Epsilon:          |-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|
% GNSS time vector:  --|------|------|------|------|------|------|
%                      G1     G2     G4     G5     G6     G7     G8
% KF execution:               ^      ^      ^             ^      ^
%
% It can be seen that the KF is not execute at G1 and G6 because of a wrong choice of epsilon.

% A rule of thumb for choosing eps is:
gnss.eps = mean(diff(imu1.t)) / 3;

%% GNSS SYNTHETIC DATA

rng('shuffle')                  % Pseudo-random seed reset
GNSS_DATA = 'ON';
if strcmp(GNSS_DATA, 'ON')      % If simulation of GNSS data is required...
    
    fprintf('NaveGo: generating GNSS synthetic data... \n')
    
    gnss = gnss_m2r(ref.lat(1), ref.h(1), gnss); % GNSS manufacturer error units to SI units
    
    gnss = gnss_gen(ref, gnss);  % Generation of GNSS dataset from reference dataset
    
    save ../gnss.mat gnss    
else
    
    fprintf('NaveGo: loading GNSS synthetic data... \n')
    
    load ../gnss.mat
end

%% IMU1 SYNTHETIC DATA

rng('shuffle')                  % Pseudo-random seed reset
IMU1_DATA = 'ON'
if strcmp(IMU1_DATA, 'ON')      % If simulation of IMU1 data is required...
    
    fprintf('NaveGo: generating IMU1 ACCR synthetic data... \n')
    
    fb = acc_gen (ref, imu1);   % Generation of acc in the body frame
    imu1.fb = fb;
    
    fprintf('NaveGo: generating IMU1 GYRO synthetic data... \n')
    
    wb = gyro_gen (ref, imu1);  % Generation of gyro in the body frame
    imu1.wb = wb;
    
    save ../imu1.mat imu1
    
    clear wb fb;    
else
    
    fprintf('NaveGo: loading IMU1 synthetic data... \n')
    
    load imu1.mat
end


%% NAVIGATION TIME

to = (ref.t(end) - ref.t(1));

fprintf('NaveGo: navigation time is %.2f minutes or %.2f seconds. \n', (to/60), to)
%% INS/GNSS INTEGRATION USING IMU1
IMU1_INS = 'ON';
if strcmp(IMU1_INS, 'ON')
    
    fprintf('NaveGo: processing INS/GNSS integration for IMU1... \n')
    
    % INS/GNSS integration
    % ---------------------------------------------------------------------
    nav1_e = ins_gnss(imu1, gnss, 'dcm');           % Attitude will be estimated by the DCM equations
    % ---------------------------------------------------------------------
    
    save nav1_e.mat nav1_e    
else
    
    fprintf('NaveGo: loading INS/GNSS integration for IMU1... \n')
    
    load nav1_e.mat
end

%% INTERPOLATION OF INS/GNSS DATASET

% INS/GNSS estimates and GNSS data are interpolated according to the
% reference dataset.

[nav1_i, ref_n1] = navego_interpolation (nav1_e, ref);
[gnss_i, ref_g] = navego_interpolation (gnss, ref);

%% NAVIGATION RMSE FOR IMU1

print_rmse (nav1_i, gnss_i, ref_n1, ref_g, 'ADIS16405 INS/GNSS');


%% PERFORMANCE ANAYSYS OF THE KALMAN FILTER

fprintf('\nNaveGo: Kalman filter performance analysis for IMU 2...\n')

%% PLOTS
PLOT = 'ON';
if (strcmp(PLOT,'ON'))
    
    % Colors
    blue = [0, 0.4470, 0.7410];
    orange = [0.8500, 0.3250, 0.0980];
%     yellow = [0.9290, 0.6940, 0.1250];
    gray= ones(1,3) * 0.75;
    
    % Line width
    lw = 1.5;

    sig3_rr = abs(nav1_e.Pp(:, 1:16:end).^(0.5)) .* 3; % Only take diagonal elements from Pp
    
    % 3D TRAJECTORY
    figure;
    plot3(ref.lon.*R2D, ref.lat.*R2D, ref.h, '--k')
    hold on
    plot3(nav1_e.lon.*R2D, nav1_e.lat.*R2D, nav1_e.h, 'Color', blue)
    plot3(ref.lon(1).*R2D, ref.lat(1).*R2D, ref.h(1), 'or', 'MarkerSize', 10, 'LineWidth', lw)
    axis tight
    title('3D TRAJECTORY')
    xlabel('Longitude [deg]')
    ylabel('Latitude [deg]')
    zlabel('Altitude [m]')
    view(-45,25)
    legend('TRUE', 'IMU1', 'IMU2')
    grid
    
    % 2D TRAJECTORY
    figure;
    plot(ref.lon.*R2D, ref.lat.*R2D, '--k')
    hold on
    plot(nav1_e.lon.*R2D, nav1_e.lat.*R2D, 'Color', blue)
    plot(ref.lon(1).*R2D, ref.lat(1).*R2D, 'or', 'MarkerSize', 10, 'LineWidth', lw)
    axis tight
    title('2D TRAJECTORY')
    xlabel('Longitude [deg]')
    ylabel('Latitude [deg]')
    legend('TRUE', 'IMU1', 'IMU2')
    grid
    
    % ATTITUDE
    figure;
    subplot(311)
    plot(ref.t, R2D.*ref.roll, '--k')
    hold on
    plot(nav1_e.t, R2D.*nav1_e.roll,'Color', blue, 'LineWidth', lw)
    ylabel('[deg]')
    xlabel('Time [s]')
    legend('REF', 'IMU1', 'IMU2');
    title('ROLL');
    grid
    
    subplot(312)
    plot(ref.t, R2D.*ref.pitch, '--k') 
    hold on
    plot(nav1_e.t, R2D.*nav1_e.pitch,'Color', blue, 'LineWidth', lw)
    ylabel('[deg]')
    xlabel('Time [s]')
    title('PITCH');
    grid
    
    subplot(313)
    plot(ref.t, R2D.* ref.yaw, '--k') 
    hold on
    plot(nav1_e.t, R2D.*nav1_e.yaw,'Color', blue, 'LineWidth', lw)
    ylabel('[deg]')
    xlabel('Time [s]')
    title('YAW');
    grid
    
    % ATTITUDE ERRORS
    figure;
    subplot(311)
    plot(nav1_e.t, (nav1_i.roll - ref_n1.roll).*R2D, 'Color', blue, 'LineWidth', lw)
    hold on
    plot (gnss.t, R2D.*sig3_rr(:,1), '--k', gnss.t, -R2D.*sig3_rr(:,1), '--k' )
    ylabel('[deg]')
    xlabel('Time [s]')
    legend('IMU1', 'IMU2', '3\sigma for IMU1');
    title('ROLL ERROR');
    grid
    
    subplot(312)
    plot(nav1_e.t, (nav1_i.pitch - ref_n1.pitch).*R2D, 'Color', blue, 'LineWidth', lw) 
    hold on
    plot (gnss.t, R2D.*sig3_rr(:,2), '--k', gnss.t, -R2D.*sig3_rr(:,2), '--k' )
    ylabel('[deg]')
    xlabel('Time [s]')
    title('PITCH ERROR');
    grid
    
    subplot(313)
    plot(nav1_e.t, (nav1_i.yaw - ref_n1.yaw).*R2D, 'Color', blue, 'LineWidth', lw)
    hold on
    plot(gnss.t, R2D.*sig3_rr(:,3), '--k', gnss.t, -R2D.*sig3_rr(:,3), '--k')
    ylabel('[deg]')
    xlabel('Time [s]')
    title('YAW ERROR');
    grid
    
    % VELOCITIES
    figure;
    subplot(311)
    plot(ref.t, ref.vel(:,1), '--k') 
    hold on
    plot(gnss.t, gnss.vel(:,1), '.', 'Color', gray, 'LineWidth', lw)
    plot(nav1_e.t, nav1_e.vel(:,1),'Color', blue, 'LineWidth', lw)
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('REF', 'GNSS', 'IMU1', 'IMU2');
    title('NORTH VELOCITY');
    grid
    
    subplot(312)
    plot(ref.t, ref.vel(:,2), '--k') 
    hold on
    plot(gnss.t, gnss.vel(:,2), '.', 'Color', gray, 'LineWidth', lw)
    plot(nav1_e.t, nav1_e.vel(:,2),'Color', blue, 'LineWidth', lw)
    xlabel('Time [s]')
    ylabel('[m/s]')
    title('EAST VELOCITY');
    grid
    
    subplot(313)
    plot(ref.t, ref.vel(:,3), '--k') 
    hold on
    plot(gnss.t, gnss.vel(:,3), '.', 'Color', gray, 'LineWidth', lw)
    plot(nav1_e.t, nav1_e.vel(:,3),'Color', blue, 'LineWidth', lw)
    xlabel('Time [s]')
    ylabel('[m/s]')
    title('DOWN VELOCITY');
    grid
    
    % VELOCITIES ERRORS
    figure;
    subplot(311)
    plot(gnss_i.t, (gnss_i.vel(:,1) - ref_g.vel(:,1)), '.', 'Color', gray, 'LineWidth', lw)
    hold on
    plot(nav1_i.t, (nav1_i.vel(:,1) - ref_n1.vel(:,1)), 'Color', blue, 'LineWidth', lw)
    plot(gnss.t, sig3_rr(:,4), '--k', gnss.t, -sig3_rr(:,4), '--k')
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('GNSS', 'IMU1', 'IMU2', '3\sigma for IMU1');
    title('VELOCITY NORTH ERROR');
    grid
    
    subplot(312)
    plot(gnss_i.t, (gnss_i.vel(:,2) - ref_g.vel(:,2)), '.', 'Color', gray, 'LineWidth', lw)
    hold on
    plot(nav1_i.t, (nav1_i.vel(:,2) - ref_n1.vel(:,2)), 'Color', blue, 'LineWidth', lw)
    plot (gnss.t, sig3_rr(:,5), '--k', gnss.t, -sig3_rr(:,5), '--k')
    xlabel('Time [s]')
    ylabel('[m/s]')
    title('VELOCITY EAST ERROR');
    grid
    
    subplot(313)
    plot(gnss_i.t, (gnss_i.vel(:,3) - ref_g.vel(:,3)), '.', 'Color', gray, 'LineWidth', lw)
    hold on
    plot(nav1_i.t, (nav1_i.vel(:,3) - ref_n1.vel(:,3)), 'Color', blue, 'LineWidth', lw)
    plot (gnss.t, sig3_rr(:,6), '--k', gnss.t, -sig3_rr(:,6), '--k')
    xlabel('Time [s]')
    ylabel('[m/s]')
    title('VELOCITY DOWN ERROR');
    grid
    
    % POSITION
    figure;
    subplot(311)
    plot(ref.t, ref.lat .*R2D, '--k') 
    hold on
    plot(gnss.t, gnss.lat.*R2D, '.', 'Color', gray, 'LineWidth', lw)
    plot(nav1_e.t, nav1_e.lat.*R2D, 'Color', blue, 'LineWidth', lw)
    xlabel('Time [s]')
    ylabel('[deg]')
    legend('REF', 'GNSS', 'IMU1', 'IMU2');
    title('LATITUDE');
    grid
    
    subplot(312)
    plot(ref.t, ref.lon .*R2D, '--k') 
    hold on
    plot(gnss.t, gnss.lon.*R2D, '.', 'Color', gray, 'LineWidth', lw)
    plot(nav1_e.t, nav1_e.lon.*R2D, 'Color', blue, 'LineWidth', lw)
    xlabel('Time [s]')
    ylabel('[deg]')
    title('LONGITUDE');
    grid
    
    subplot(313)
    plot(ref.t, ref.h, '--k') 
    hold on
    plot(gnss.t, gnss.h, '.', 'Color', gray, 'LineWidth', lw)
    plot(nav1_e.t, nav1_e.h, 'Color', blue, 'LineWidth', lw)
    xlabel('Time [s]')
    ylabel('[m]')
    title('ALTITUDE');
    grid
    
    % POSITION ERRORS
    [RN,RE]  = radius(nav1_i.lat);
    LAT2M_1 = RN + nav1_i.h;
    LON2M_1 = (RE + nav1_i.h).*cos(nav1_i.lat);
    
    [RN,RE]  = radius(gnss.lat);
    LAT2M_G = RN + gnss.h;
    LON2M_G = (RE + gnss.h).*cos(gnss.lat);
    
    [RN,RE]  = radius(gnss_i.lat);
    LAT2M_GR = RN + gnss_i.h;
    LON2M_GR = (RE + gnss_i.h).*cos(gnss_i.lat);
    
    figure;
    subplot(311)
    plot(gnss_i.t,  LAT2M_GR.*(gnss_i.lat - ref_g.lat), '.', 'Color', gray, 'LineWidth', lw)
    hold on
    plot(nav1_i.t, LAT2M_1.*(nav1_i.lat - ref_n1.lat), 'Color', blue, 'LineWidth', lw)
    plot (gnss.t, LAT2M_G.*sig3_rr(:,7), '--k', gnss.t, -LAT2M_G.*sig3_rr(:,7), '--k' )
    xlabel('Time [s]')
    ylabel('[m]')
    legend('GNSS', 'IMU1', 'IMU2', '3\sigma for IMU1');
    title('LATITUDE ERROR');
    grid
    
    subplot(312)
    plot(gnss_i.t, LON2M_GR.*(gnss_i.lon - ref_g.lon), '.', 'Color', gray, 'LineWidth', lw)
    hold on
    plot(nav1_i.t, LON2M_1.*(nav1_i.lon - ref_n1.lon), 'Color', blue, 'LineWidth', lw)
    plot(gnss.t, LON2M_G.*sig3_rr(:,8), '--k', gnss.t, -LON2M_G.*sig3_rr(:,8), '--k' )
    xlabel('Time [s]')
    ylabel('[m]')
    title('LONGITUDE ERROR');
    grid
    
    subplot(313)
    plot(gnss_i.t, (gnss_i.h - ref_g.h), '.', 'Color', gray, 'LineWidth', lw)
    hold on
    plot(nav1_i.t, (nav1_i.h - ref_n1.h), 'Color', blue, 'LineWidth', lw)
    plot(gnss.t, sig3_rr(:,9), '--k', gnss.t, -sig3_rr(:,9), '--k' )
    xlabel('Time [s]')
    ylabel('[m]')
    title('ALTITUDE ERROR');
    grid
    
    % BIAS ESTIMATION
    figure;
    subplot(311)
    plot(nav1_e.tg, nav1_e.b(:, 1).*R2D, 'Color', blue, 'LineWidth', lw)
    hold on
    plot(nav1_e.tg, sig3_rr(:,10).*R2D, '--k', nav1_e.tg, -sig3_rr(:,10).*R2D, '--k' )
    hold off
    xlabel('Time [s]')
    ylabel('[deg]')
    title('KF BIAS GYRO X ESTIMATION');
    legend('IMU1', 'IMU2', '3\sigma for IMU1');
    grid
    
    subplot(312)
    plot(nav1_e.tg, nav1_e.b(:, 2).*R2D, 'Color', blue, 'LineWidth', lw)
    hold on
    plot(nav1_e.tg, sig3_rr(:,11).*R2D, '--k', nav1_e.tg, -sig3_rr(:,11).*R2D, '--k' )
    hold off
    xlabel('Time [s]')
    ylabel('[deg]')
    title('KF BIAS GYRO Y ESTIMATION');
    grid
    
    subplot(313)
    plot(nav1_e.tg, nav1_e.b(:, 3).*R2D, 'Color', blue, 'LineWidth', lw)
    hold on
    plot(nav1_e.tg, sig3_rr(:,12).*R2D, '--k', nav1_e.tg, -sig3_rr(:,12).*R2D, '--k' )
    hold off
    xlabel('Time [s]')
    ylabel('[deg]')
    title('KF BIAS GYRO Z ESTIMATION');
    grid
    
    figure;
    subplot(311)
    plot(nav1_e.tg, nav1_e.b(:, 4), 'Color', blue, 'LineWidth', lw)
    hold on
    plot(nav1_e.tg, sig3_rr(:,13), '--k', nav1_e.tg, -sig3_rr(:,13), '--k' )
    hold off
    xlabel('Time [s]')
    ylabel('[m/s^2]')
    title('KF BIAS ACCR X ESTIMATION');
    legend('IMU1', 'IMU2', '3\sigma for IMU1');
    grid
    
    subplot(312)
    plot(nav1_e.tg, nav1_e.b(:, 5), 'Color', blue, 'LineWidth', lw)
    hold on
    plot(nav1_e.tg, sig3_rr(:,14), '--k', nav1_e.tg, -sig3_rr(:,14), '--k' )
    hold off
    xlabel('Time [s]')
    ylabel('[m/s^2]')
    title('KF BIAS ACCR Y ESTIMATION');
    grid
    
    subplot(313)
    plot(nav1_e.tg, nav1_e.b(:, 6), 'Color', blue, 'LineWidth', lw)
    hold on
    plot(nav1_e.tg, sig3_rr(:,15), '--k', nav1_e.tg, -sig3_rr(:,15), '--k' )
    hold off
    xlabel('Time [s]')
    ylabel('[m/s^2]')
    title('KF BIAS ACCR Z ESTIMATION');
    grid
end
