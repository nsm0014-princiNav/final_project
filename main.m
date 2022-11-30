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
IMU.t = turn_data.memsense.imuTimeReference.gpsSeconds';                    % t: Ix1 time vector [s]
IMU.fb = turn_data.memsense.imuTimeReference.linearAcceleration';           % fb: Ix3 accelerations vector in body frame XYZ (m/s^2).
IMU.wb = turn_data.memsense.imuTimeReference.angularVelocity';              % wb: Ix3 turn rates vector in body frame XYZ (radians/s).
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
GPS.t = turn_data.novatel_local.gpsTimeTagged.gpsSeconds';                  % t: Gx1 time vector (seconds).
GPS.lat = deg2rad(turn_data.novatel_local.odom.positionLla(1,:)');          % lat: Gx1 latitude (radians).
GPS.lon = deg2rad(turn_data.novatel_local.odom.positionLla(2,:)');          % lon: Gx1 longitude (radians).
GPS.h = turn_data.novatel_local.odom.positionLla(3,:)';                     % h: Gx1 altitude (m).
GPS.vel = turn_data.novatel_local.odom.velocityNed';                        % vel: Gx3 NED velocities (m/s)
GPS.std = GPSbiases.std;                                                    % std: 1x3 position standard deviations (rad, rad, m)
GPS.stdm = GPSbiases.stdm;                                                  % stdm: 1x3 position standard deviations (m, m, m)
GPS.stdv = GPSbiases.stdv;                                                  % stdv: 1x3 velocity standard deviations (m/s)
GPS.larm = [0;0;0];                                                         % larm: 3x1 lever arm from IMU to GNSS antenna (x-fwd, y-right, z-down) (m)
GPS.freq = 10;                                                              % freq: 1x1 sampling frequency (Hz)
GPS.eps = 0.05;                                                             % eps: 1x1 time interval to compare current IMU time to current GNSS time vector (s)

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
qua   = eul2quat([estiYaw(1) estiPitch(1) estiRoll(1)]);

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


a = 6378137.0;                  % WGS84 Equatorial radius in meters
e = 0.0818191908425;            % WGS84 eccentricity

e2 = e^2;
den = 1 - e2 .* (sin(GPS.lat(1))).^2;

% Meridian radius of curvature: radius of curvature for North-South motion.
RM = a .* (1-e2) ./ (den).^(3/2);

% Normal radius of curvature: radius of curvature for East-West motion.
% AKA transverse radius.
RN = a ./ sqrt(den);

Tpr = diag([(RM + GPS.h(1)), (RN + GPS.h(1)) * cos(GPS.lat(1)), -1]);  % radians-to-meters

% Update matrix H
kf.H = [ O I O O O ;
    O O Tpr O O ; ];
kf.R = diag([GPS.stdv GPS.stdm]).^2;
kf.z = [ GPS.stdv, GPS.stdm ]';

% Propagate prior estimates to get xp(1) and Pp(1)
kf = filter.kf_update(kf);

% Initial matrices for Kalman filter performance analysis
xi(1,:) = kf.xi';
xp(1,:) = kf.xp';
Pi(1,:) = reshape(kf.Pi, 1, numStates^2);
Pp(1,:) = reshape(kf.Pp, 1, numStates^2);
K(1,:)  = reshape(kf.K, 1, numStates*numSensors);
S(1,:)  = reshape(kf.S, 1, numSensors^2);
v(1,:)  = kf.v';
z(1,:)  = kf.z';
b(1,:) = [gb_dyn', ab_dyn'];

%% Beginning Loosley Coupled Kalman Filter
for i = 2:numIter_imu

    % IMU sampling interval
    dti = IMU.t(i) - IMU.t(i-1);

    % Inertial sensors corrected with a posteriori KF biases estimation and
    % deterministic static biases
    wb_corrected = IMU.wb(i,:)' - gb_dyn - IMU.gb_sta';
    fb_corrected = IMU.fb(i,:)' - ab_dyn - IMU.ab_sta';
    fn = C_b_n * fb_corrected;
    wn = C_b_n * wb_corrected;

    % Velocity update
    vel = imu.vel_update(fn, estiVelo(i-1,:), omega_ie_n, omega_en_n, gn_e(i-1,:)', dti);
    estiVelo (i,:) = vel;

    % Position update
    pos = imu.pos_update([estiLat(i-1) estiLong(i-1) estiAlt(i-1)], estiVelo(i,:), dti);
    estiLat(i) = pos(1);
    estiLong(i) = pos(2);
    estiAlt(i)   = pos(3);

    % Turn-rates update with both updated velocity and position
    omega_ie_n = [       0        , sin(estiLat(i)),       0        ; ...
        -sin(estiLat(i)),    0           ,-cos(estiLat(i)); ...
        0        , cos(estiLat(i)),       0        ].*(7.2921155e-5);
    omega_en_n = imu.transportRate(estiLat(i), estiVelo(i,1), estiVelo(i,2), estiAlt(i));

    % Gravity update
    gn_e(i,:) = imu.gravity(estiLat(i), estiAlt(i));

    % Attitude update
    [qua, C_b_n, euler] = imu.att_update(wb_corrected, C_b_n, qua, ...
        omega_ie_n, omega_en_n, dti, 'dcm');
    estiRoll(i) = euler(1);
    estiPitch(i) = euler(2);
    estiYaw(i) = euler(3);

    %% KALMAN FILTER UPDATE

    % Check if there is a new GNSS measurement to process at current INS time
    gdx =  find(GPS.t >= (IMU.t(i) - GPS.eps) & GPS.t < (IMU.t(i) + GPS.eps));

    if ( ~isempty(gdx) && gdx > 1)

        %% MEASUREMENTS

        % Meridian and normal radii of curvature update
        a = 6378137.0;                  % WGS84 Equatorial radius in meters
        e = 0.0818191908425;            % WGS84 eccentricity

        e2 = e^2;
        den = 1 - e2 .* (sin(estiLat(i))).^2;

        % Meridian radius of curvature: radius of curvature for North-South motion.
        RM = a .* (1-e2) ./ (den).^(3/2);

        % Normal radius of curvature: radius of curvature for East-West motion.
        % AKA transverse radius.
        RN = a ./ sqrt(den);

        % Radians-to-meters matrix
        Tpr = diag([(RM + estiAlt(i)), (RN + estiAlt(i)) * cos(estiLat(i)), -1]);

        % Position innovations in meters with lever arm correction
        zp = Tpr * ([estiLat(i); estiLong(i); estiAlt(i);] - [GPS.lat(gdx); GPS.lon(gdx); GPS.h(gdx);]) ...
            + (C_b_n * GPS.larm);

        % Velocity innovations with lever arm correction
        zv = (estiVelo(i,:) - GPS.vel(gdx,:) - ((omega_ie_n + omega_en_n) * (C_b_n * GPS.larm ))' ...
            + (C_b_n * formskewsym(wb_corrected) * GPS.larm)')';

        %% KALMAN FILTER

        % GNSS sampling interval
        dtg = GPS.t(gdx) - GPS.t(gdx-1);

        % Vector to update matrix F
        upd = [estiVelo(i,:) estiLat(i) estiAlt(i) fn' wn'];

        % Matrices F and G update
        [kf.F, kf.G] = filter.F_update(upd, C_b_n, IMU);

        % Matrix H update
        kf.H = [ O I O O O ; ];
        kf.R = diag([GPS.stdv]).^2;
        kf.z = zv;

        % a posteriori states are forced to be zero (error-state approach)
        kf.xp = zeros(numStates , 1);
        % Execution of the extended Kalman filter
        kf = filter.kalman(kf, dtg);

        %% OBSERVABILITY

        % Number the observable states at current GNSS time
        ob(gdx) = rank(obsv(kf.F, kf.H));

        %% INS/GNSS CORRECTIONS

        % Quaternion correction
        qua_skew = -formskewsym(qua(1:3));    % According to Crassidis, qua_skew should be
        % positive, but if positive NaveGo diverges.
        % Crassidis, Eq. A.174a
        Xi = [qua(4)*eye(3) + qua_skew; -qua(1:3)'];

        % Crassidis, Eq. 7.34
        qua = qua + 0.5 .* Xi * kf.xp(1:3);
        qua = qua / norm(qua);          % Brute-force normalization

        % DCM correction
        C_b_n = quat2dcm(qua');

        % Attitude correction, method 2
        estiRoll(i)  = estiRoll(i)  - kf.xp(1);
        estiPitch(i) = estiPitch(i) - kf.xp(2);
        estiYaw(i)   = estiYaw(i)   - kf.xp(3);

        % Velocity correction
        estiVelo(i,1) = estiVelo(i,1) - kf.xp(4);
        estiVelo(i,2) = estiVelo(i,2) - kf.xp(5);
        estiVelo(i,3) = estiVelo(i,3) - kf.xp(6);

        % Position correction
        estiLat(i) = estiLat(i) - kf.xp(7);
        estiLong(i) = estiLong(i) - kf.xp(8);
        estiAlt(i)   = estiAlt(i)   - kf.xp(9);

        % Biases estimation
        gb_dyn   = -kf.xp(10:12);
        ab_dyn   = -kf.xp(13:15);

        % Matrices for later Kalman filter performance analysis
        xi(gdx,:) = kf.xi';
        xp(gdx,:) = kf.xp';
        b(gdx,:) = [gb_dyn', ab_dyn'];
        A(gdx,:)  = reshape(kf.A,  1, numStates^2);
        Pi(gdx,:) = reshape(kf.Pi, 1, numStates^2);
        Pp(gdx,:) = reshape(kf.Pp, 1, numStates^2);

        z(gdx,:)  = [ kf.z' 0 0 0 ]';
        v(gdx,:)  = [ kf.v' 0 0 0 ]';
        K(gdx,1:numStates*3) = reshape(kf.K, 1, numStates*3);
        S(gdx,1:9)  = reshape(kf.S, 1, 3^2);

    end
end

%% Summary from INS/GNSS integration

LC_Sol.t     = IMU.t(1:i, :);    % INS time vector
LC_Sol.tg    = GPS.t;           % GNSS time vector, which is the time vector when the Kalman filter was executed
LC_Sol.roll  = estiRoll(1:i, :);   % Roll
LC_Sol.pitch = estiPitch(1:i, :);  % Pitch
LC_Sol.yaw   = estiYaw(1:i, :);    % Yaw
LC_Sol.vel   = estiVelo(1:i, :);    % NED velocities
LC_Sol.lat   = estiLat(1:i, :);    % Latitude
LC_Sol.lon   = estiLong(1:i, :);    % Longitude
LC_Sol.h     = estiAlt(1:i, :);      % Altitude
LC_Sol.gn    = gn_e(1:i, :);     % Gravity estimation in the nav-frame.

LC_Sol.xi    = xi;       % A priori states
LC_Sol.xp    = xp;       % A posteriori states
LC_Sol.z     = z;        % INS/GNSS measurements
LC_Sol.v     = v;        % Kalman filter innovations
LC_Sol.b     = b;        % Biases compensations

LC_Sol.A     = A;        % Transition matrices
LC_Sol.Pi    = Pi;       % A priori covariance matrices
LC_Sol.Pp    = Pp;       % A posteriori covariance matrices
LC_Sol.K     = K;        % Kalman gain matrices
LC_Sol.S     = S;        % Innovation matrices
LC_Sol.ob    = ob;       % Number of observable states after each GNSS data arriving

%% Plotting
geoplot(rad2deg(LC_Sol.lat),rad2deg(LC_Sol.lon))
geobasemap satellite
