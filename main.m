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

%% Defining Inputs
% Calculating biases, random walks, and noise parameters from static data
[IMUbiases,GPSbiases] = imu.calcBias(static_data);

% IMU - TURN CASE
IMU.t = turn_data.memsense.imu.time';                                       % t: Ix1 time vector [s]
IMU.fb = turn_data.memsense.imu.linearAcceleration';                        % fb: Ix3 accelerations vector in body frame XYZ (m/s^2).
IMU.wb = turn_data.memsense.imu.angularVelocity';                           % wb: Ix3 turn rates vector in body frame XYZ (radians/s).
IMU.arw = IMUbiases.arw;                                                    % arw: 1x3 angle random walks (rad/s/root-Hz).
IMU.vrw = IMUbiases.vrw;                                                    % vrw: 1x3 velocity random walks (m/s^2/root-Hz).
IMU.g_std = IMUbiases.g_std;                                                % g_std: 1x3 gyros standard deviations (radians/s).
IMU.a_std = IMUbiases.a_std;                                                % a_std: 1x3 accrs standard deviations (m/s^2).
IMU.gb_sta = IMUbiases.gb_sta;                                              % gb_sta: 1x3 gyros static biases or turn-on biases (radians/s).
IMU.ab_sta = IMUbiases.ab_sta;                                              % ab_sta: 1x3 accrs static biases or turn-on biases (m/s^2).
IMU.gb_dyn = IMUbiases.gb_dyn;                                              % gb_dyn: 1x3 gyros dynamic biases or bias instabilities (radians/s).
IMU.ab_dyn = IMUbiases.ab_dyn;                                              % ab_dyn: 1x3 accrs dynamic biases or bias instabilities (m/s^2).
IMU.gb_corr = IMUbiases.gb_corr;                                            % gb_corr: 1x3 gyros correlation times (seconds).
IMU.ab_corr = IMUbiases.ab_corr;                                            % ab_corr: 1x3 accrs correlation times (seconds).
IMU.gb_psd = IMUbiases.gb_psd;                                              % gb_psd: 1x3 gyros dynamic biases root-PSD (rad/s/root-Hz).
IMU.ab_psd = IMUbiases.ab_psd;                                              % ab_psd: 1x3 accrs dynamic biases root-PSD (m/s^2/root-Hz);
IMU.freq = 100;                                                             % freq: 1x1 sampling frequency (Hz).
IMU.ini_align = [-0.015012 -0.022386 -1.726];                               % ini_align: 1x3 initial attitude at t(1).
IMU.ini_align_error = [0 0 0];                                              % ini_align_err: 1x3 initial attitude errors at t(1).

% GPS - TURN CASE
GPS.t = turn_data.novatel_local.odom.time';                                 % t: Gx1 time vector (seconds).
GPS.lat = deg2rad(turn_data.novatel_local.odom.positionLla(1,:)');          % lat: Gx1 latitude (radians).
GPS.lon = deg2rad(turn_data.novatel_local.odom.positionLla(2,:)');          % lon: Gx1 longitude (radians).
GPS.h = turn_data.novatel_local.odom.positionLla(3,:)';                     % h: Gx1 altitude (m).
GPS.vel = turn_data.novatel_local.odom.velocityNed';                        % vel: Gx3 NED velocities (m/s)
GPS.std = GPSbiases.std;                                                    % std: 1x3 position standard deviations (rad, rad, m)
GPS.stdm = GPSbiases.stdm;                                                  % stdm: 1x3 position standard deviations (m, m, m)
GPS.stdv = GPSbiases.stdv;                                                  % stdv: 1x3 velocity standard deviations (m/s)
GPS.larm = [0;0;0];                                                         % larm: 3x1 lever arm from IMU to GNSS antenna (x-fwd, y-right, z-down) (m)
GPS.freq = 10;                                                              % freq: 1x1 sampling frequency (Hz)
GPS.eps = 0.01;                                                             % eps: 1x1 time interval to compare current IMU time to current GNSS time vector (s)

%% Pre-Allocating Filter Inputs/Outputs

% Kalman filter dimensions
numStates = 15;                                                            
numSensors = 6;                                                            

% Constant matrices
I = eye(3);
O = zeros(3);

numIter_imu = length(IMU.t);                                               
numIter_gps = length(GPS.t);                                                

% Attitude
estiRoll = zeros(numIter_imu, 1);
estiPitch = zeros(numIter_imu, 1);
estiYaw = zeros(numIter_imu, 1);

% Velocity
estiVelo = zeros(numIter_imu, 3);

% Gravity
gn_e = zeros (numIter_imu, 3);

% Position Vectors
estiLat = zeros (numIter_imu, 1);
estiLong = zeros (numIter_imu, 1);
estiAlt = zeros (numIter_imu, 1);

% Kalman filter Parameters
xi = zeros(numIter_gps, numStates);                                         % Evolution of Kalman filter a priori states
xp = zeros(numIter_gps, numStates);                                         % Evolution of Kalman filter a posteriori states
z = zeros(numIter_gps, numSensors);                                         % INS/GNSS measurements
v = zeros(numIter_gps, numSensors);                                         % Kalman filter innovations

A  = zeros(numIter_gps, numStates^2);                                       % Transition-state matrices
Pi = zeros(numIter_gps, numStates^2);                                       % A priori covariance matrices
Pp = zeros(numIter_gps, numStates^2);                                       % A posteriori covariance matrices
K  = zeros(numIter_gps, numStates*numSensors);                              % Kalman gain matrices
S  = zeros(numIter_gps, numSensors^2);                                      % Innovation matrices
ob = zeros(numIter_gps, numSensors);                                        % Number of observable states at each GNSS data arriving

b = zeros(numIter_gps, numSensors);                                         % Biases compensantions after Kalman filter correction

%% Initializing Loosley Couple GPS/INS Algorithm

% Initial attitude
estiRoll(1)  = IMU.ini_align(1);
estiPitch(1) = IMU.ini_align(2);
estiYaw(1)   = IMU.ini_align(3);
C_n_b = genDCM('rad',[estiYaw(1) estiPitch(1) estiRoll(1)],[3 2 1]);
C_b_n = C_n_b';

% Initial velocity
estiVelo(1,:) = GPS.vel(1,:);

% Initial position
estiLat(1) = GPS.lat(1);
estiLong(1) = GPS.lon(1);
estiAlt(1)   = GPS.h(1);

% Initial dynamic biases
gb_dyn = IMU.gb_dyn';
ab_dyn = IMU.ab_dyn';

% Turn-rates update with both updated velocity and position
omega_ie_n = [       0        , sin(estiLat(1)),       0        ; ...
              -sin(estiLat(1)),    0           ,-cos(estiLat(1)); ...
                     0        , cos(estiLat(1)),       0        ].*(7.2921155e-5);

omega_en_n = imu.transportRate(estiLat(1), estiVelo(1,1), estiVelo(1,2), estiAlt(1));

% Gravity update
gn_e(1,:) = imu.gravity(estiLat(1), estiAlt(1));

% Prior estimates
kf.xi = [zeros(1,9), IMU.gb_dyn, IMU.ab_dyn ]';  % Error vector state
kf.Pi = diag([IMU.ini_align_error, GPS.stdv, GPS.std, IMU.gb_dyn, IMU.ab_dyn].^2);

kf.Q  = diag([IMU.arw, IMU.vrw, IMU.gb_psd, IMU.ab_psd].^2);

fn = C_b_n*(IMU.fb(1,:)' - ab_dyn - IMU.ab_sta');
wn = C_b_n*(IMU.wb(1,:)' - gb_dyn - IMU.gb_sta');

% Vector to update matrix F
upd = [GPS.vel(1,:) GPS.lat(1) GPS.h(1) fn' wn'];

% Update matrices F and G
[kf.F, kf.G] = filter.F_update(upd, C_b_n, IMU);

[RM,RN] = radius(gnss.lat(1));
Tpr = diag([(RM + gnss.h(1)), (RN + gnss.h(1)) * cos(gnss.lat(1)), -1]);  % radians-to-meters

% Update matrix H
kf.H = [ O I O O O ;
    O O Tpr O O ; ];
kf.R = diag([gnss.stdv gnss.stdm]).^2;
kf.z = [ gnss.stdv, gnss.stdm ]';

% Propagate prior estimates to get xp(1) and Pp(1)
kf = kf_update( kf );

% Initial matrices for Kalman filter performance analysis
xi(1,:) = kf.xi';
xp(1,:) = kf.xp';
Pi(1,:) = reshape(kf.Pi, 1, n^2);
Pp(1,:) = reshape(kf.Pp, 1, n^2);
K(1,:)  = reshape(kf.K, 1, n*r);
S(1,:)  = reshape(kf.S, 1, r^2);
v(1,:)  = kf.v';
z(1,:)  = kf.z';
b(1,:) = [gb_dyn', ab_dyn'];
