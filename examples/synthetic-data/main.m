%% Formatting
clc
close all
clear
matlabrc

addpath ../../ins/
addpath ../../ins-gnss/
addpath ../../conversions/
addpath ../../performance-analysis/
addpath ../../misc/
addpath ../../simulation/
addpath ../../plot/

navego_print_version;

fprintf('\nNaveGo: starting simulation of INS/GNSS integration... \n')

%% CODE EXECUTION PARAMETERS

% Please, comment any of the following parameters in order to NOT execute a
% particular portion of code

GNSS_DATA = 'ON';   % Generation of synthetic GNSS data
IMU1_DATA = 'ON';   % Generation of synthetic ADIS16405 IMU data
IMU2_DATA = 'ON';   % Generation of synthetic ADIS16488 IMU data

IMU1_INS  = 'ON';   % Execution of INS/GNSS integration for ADIS16405 IMU
IMU2_INS  = 'ON';   % Execution of INS/GNSS integration for ADIS16488 IMU

PLOT      = 'ON';   % Generation of plots

% If a particular parameter is commented above, it is set by default to 'OFF'.

if (~exist('GNSS_DATA','var')), GNSS_DATA = 'OFF'; end
if (~exist('IMU1_DATA','var')), IMU1_DATA = 'OFF'; end
if (~exist('IMU2_DATA','var')), IMU2_DATA = 'OFF'; end
if (~exist('IMU1_INS','var')),  IMU1_INS  = 'OFF'; end
if (~exist('IMU2_INS','var')),  IMU2_INS  = 'OFF'; end
if (~exist('PLOT','var')),      PLOT      = 'OFF'; end

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

load ref.mat

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

imu2 = imu_si_errors(ADIS16488, dt);        % Transform IMU manufacturer error units to SI units.

imu2.ini_align_err = [0.5 0.5 1] .* D2R;                 % Initial attitude align errors for matrix P in Kalman filter, [roll pitch yaw] (radians)
imu2.ini_align = [ref.roll(1) ref.pitch(1) ref.yaw(1)];  % Initial attitude align at t(1) (radians)
imu2.t = ADIS16488.t;

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
gnss.freq = 5;                          % GNSS operation frequency (Hz)

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

if strcmp(GNSS_DATA, 'ON')      % If simulation of GNSS data is required...

    fprintf('NaveGo: generating GNSS synthetic data... \n')

    gnss = gnss_m2r(ref.lat(1), ref.h(1), gnss); % GNSS manufacturer error units to SI units

    gnss = gnss_gen(ref, gnss);  % Generation of GNSS dataset from reference dataset

    save gnss.mat gnss
else

    fprintf('NaveGo: loading GNSS synthetic data... \n')

    load gnss.mat
end

%% IMU1 SYNTHETIC DATA

rng('shuffle')                  % Pseudo-random seed reset

    fprintf('NaveGo: generating IMU1 ACCR synthetic data... \n')

    fb = acc_gen (ref, imu1);   % Generation of acc in the body frame
    imu1.fb = fb;

    fprintf('NaveGo: generating IMU1 GYRO synthetic data... \n')

    wb = gyro_gen (ref, imu1);  % Generation of gyro in the body frame
    imu1.wb = wb;

    save imu1.mat imu1

    clear wb fb;

%% NAVIGATION TIME

to = (ref.t(end) - ref.t(1));

fprintf('NaveGo: navigation time is %.2f minutes or %.2f seconds. \n', (to/60), to)

%% INS/GNSS INTEGRATION USING IMU1


    fprintf('NaveGo: processing INS/GNSS integration for IMU1... \n')

    % INS/GNSS integration
    % ---------------------------------------------------------------------
    nav1_e = ins_gnss(imu1, gnss, 'dcm');           % Attitude will be estimated by the DCM equations
    % ---------------------------------------------------------------------

    save nav1_e.mat nav1_e

%% PLOTTING

plotData(nav1_e,ref)
