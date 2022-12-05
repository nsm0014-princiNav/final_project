function estimates = LC_EKF(imu, gnss)
%% Pre-allocating arrays for speed
% Length of time
numIMUIterations = length(imu.t);
numGPSIterations = length(gnss.t);

% Attitude
estimatedRoll = [imu.ini_align(1); zeros(numIMUIterations-1, 1)];
estimatedPitch = [imu.ini_align(2); zeros(numIMUIterations-1, 1)];
estimatedYaw = [imu.ini_align(3); zeros(numIMUIterations-1, 1)];

% Velocity
estimatedVelocities = zeros(numIMUIterations, 3);

% Position
estimatedLatitude = [gnss.lat(1); zeros(numIMUIterations-1, 1)];
estimatedLongitude = [gnss.lon(1); zeros(numIMUIterations-1, 1)];
estimatedAltitude = [gnss.h(1); zeros(numIMUIterations-1, 1)];

% Gravity
estimatedGravity_Nav = [gravity(estimatedLatitude(1), estimatedAltitude(1)); zeros(numIMUIterations, 3)];

% Bias
gyrDynamicBiases = imu.gb_dyn';
accDynamicBiases = imu.ab_dyn';
biases = [gyrDynamicBiases', accDynamicBiases'; zeros(numGPSIterations-1, 6)];

% Initial Rotation
DCMnav2body = genDCM('rad',[estimatedYaw(1) estimatedPitch(1) estimatedRoll(1)],[3 2 1]);
DCMbody2nav = DCMnav2body';

% Earth's Rotation Rate
omegaEci2Ecef_Nav = earth_rate(estimatedLatitude(1));
omegaEci2Navi_Nav = transport_rate(estimatedLatitude(1), estimatedVelocities(1,1), estimatedVelocities(1,2), estimatedAltitude(1));

% A Priori Error State
kf.xi = [zeros(9,1); imu.gb_dyn'; imu.ab_dyn'];

% A Priori Covariance Matrix
kf.Pi = diag([imu.ini_align_err, gnss.stdv, gnss.std, imu.gb_dyn, imu.ab_dyn].^2);

% IMU Processing Noise Matrix
kf.Q  = diag([imu.arw, imu.vrw, imu.gb_psd, imu.ab_psd].^2);

% Converting initial body frame specific forces to nav frame
specificForce_Nav = DCMbody2nav*imu.fb(1,:)';

% Intial RM and RN
[RM,RN] = radius(estimatedLatitude(1));

% Update matrices F and G
[kf.F, kf.G] = dynamicModel(0,0,0, estimatedLatitude(1), estimatedAltitude(1),specificForce_Nav', DCMbody2nav,RM,RN);

[RM,RN] = radius(gnss.lat(1));
Tpr = diag([(RM + gnss.h(1)), (RN + gnss.h(1))*cos(gnss.lat(1)), -1]);  % radians-to-meters

% Measurement Matrix
kf.H = [zeros(3),  eye(3) , zeros(3), zeros(3), zeros(3);
    zeros(3), zeros(3),   Tpr   , zeros(3), zeros(3)];

% GPS Measurement Noise Matrix
kf.R = diag([gnss.stdv gnss.stdm]).^2;

% Innovation Matrix
kf.z = zeros(6,1);

% Propagate prior estimates to get xp(1) and Pp(1)
kf.K = (kf.Pi*kf.H')*(kf.R + kf.H*kf.Pi*kf.H')^(-1) ;			% Kalman gain matrix

% Step 4, update the a posteriori state vector, xp
kf.xp = kf.xi + kf.K*kf.z;

% Step 5, update the a posteriori covariance matrix, Pp
kf.Pp = kf.Pi - kf.K*(kf.R + kf.H * kf.Pi * kf.H')*kf.K';

for i = 2:numIMUIterations
    %% A Priori Estimates (Time Update)
    % Current time step
    timeStepIMU = imu.t(i) - imu.t(i-1);

    % Correcting IMU measurements with bias estimationp
    wb_corrected = imu.wb(i,:)' - gyrDynamicBiases - imu.gb_sta';
    fb_corrected = imu.fb(i,:)' - accDynamicBiases - imu.ab_sta';

    % Converting initial body frame specific forces to nav frame
    specificForce_Nav = DCMbody2nav*fb_corrected;

    % Velocity update
    vel = vel_update(specificForce_Nav, estimatedVelocities(i-1,:), omegaEci2Ecef_Nav, omegaEci2Navi_Nav, estimatedGravity_Nav(i-1,:)', timeStepIMU);
    estimatedVelocities (i,:) = vel;

    % Position update
    pos = pos_update([estimatedLatitude(i-1) estimatedLongitude(i-1) estimatedAltitude(i-1)], estimatedVelocities(i,:), timeStepIMU);
    estimatedLatitude(i) = pos(1);
    estimatedLongitude(i) = pos(2);
    estimatedAltitude(i) = pos(3);

    % Turn-rates update with both updated velocity and position
    omegaEci2Ecef_Nav = earth_rate(estimatedLatitude(i));
    omegaEci2Navi_Nav = transport_rate(estimatedLatitude(i), estimatedVelocities(i,1), estimatedVelocities(i,2), estimatedAltitude(i));

    % Gravity update
    estimatedGravity_Nav(i,:) = gravity(estimatedLatitude(i), estimatedAltitude(i));

    % Attitude update
    [estimatedQuaternion, DCMbody2nav, euler] = att_update(wb_corrected, DCMbody2nav, omegaEci2Ecef_Nav, omegaEci2Navi_Nav, timeStepIMU);
    estimatedRoll(i) = euler(1);
    estimatedPitch(i) = euler(2);
    estimatedYaw(i)  = euler(3);

    %% Only perform A Posteriori update if there are GPS Measurements at the current timestep
    measurementUpdateIdx = find(gnss.t >= (imu.t(i) - gnss.eps) & gnss.t < (imu.t(i) + gnss.eps));

    if ( ~isempty(measurementUpdateIdx) && measurementUpdateIdx > 1)
        % Current GPS time step (time since last mesurement update)
        timeStepGPS = gnss.t(measurementUpdateIdx) - gnss.t(measurementUpdateIdx-1);

        % Grouping current estimated position
        estimatedLLA = [estimatedLatitude(i); estimatedLongitude(i); estimatedAltitude(i)];
        measurementLLA = [gnss.lat(measurementUpdateIdx); gnss.lon(measurementUpdateIdx); gnss.h(measurementUpdateIdx)];

        % Meridian and normal radii of curvature update
        [RM,RN] = radius(estimatedLatitude(i));

        % Radians-to-meters matrix
        Tpr = diag([(RM + estimatedAltitude(i)), (RN + estimatedAltitude(i)) * cos(estimatedLatitude(i)), -1]);

        % Innovation Matrix
        kf.z = [(estimatedVelocities(i,:) - gnss.vel(measurementUpdateIdx,:))'; ...
            Tpr*(estimatedLLA - measurementLLA)];

        % Dynamic Model (F) and Noise Distribution Matrix (G)
        [kf.F, kf.G] = dynamicModel(estimatedVelocities(i,1), estimatedVelocities(i,2), estimatedVelocities(i,3), estimatedLatitude(i), estimatedAltitude(i), specificForce_Nav', DCMbody2nav,RM,RN);

        % Matrix H update
        kf.H = [zeros(3),  eye(3) , zeros(3), zeros(3), zeros(3);
            zeros(3), zeros(3),   Tpr   , zeros(3), zeros(3)];

        % GPS Measurement Noise Matrix
        kf.R = diag([gnss.stdv gnss.stdm]).^2;

        % a posteriori states are forced to be zero (error-state approach)
        kf.xp = zeros(15 , 1);

        % Dynamic Model in Discrete
        kf.A =  expm(kf.F * timeStepGPS);

        % IMU Processing Noise Matrix
        kf.Qd = (kf.G * kf.Q * kf.G') .* timeStepGPS;

        % A Priori Error State Matrix
        kf.xi = zeros(15, 1);

        % A Priori Covariance Matrix
        kf.Pi = (kf.A * kf.Pp * kf.A') + kf.Qd;

        % Kalman Gain
        kf.K = (kf.Pi * kf.H') * (kf.R + kf.H * kf.Pi * kf.H')^(-1) ;			% Kalman gain matrix

        % A Postierori Error State Vector
        kf.xp = kf.K * kf.z;

        % A Postierori Covariance Matrix
        kf.Pp = kf.Pi - kf.K * (kf.R + kf.H * kf.Pi * kf.H') * kf.K';

        %% Correcting estimates with A Posteriori error state matrix xp
        % Quaternion
        qua_skew = -formskewsym(estimatedQuaternion(1:3));
        Xi = [estimatedQuaternion(4)*eye(3) + qua_skew; -estimatedQuaternion(1:3)'];
        estimatedQuaternion = estimatedQuaternion + 0.5 .* Xi * kf.xp(1:3);
        estimatedQuaternion = estimatedQuaternion / norm(estimatedQuaternion);

        % DCM
        DCMbody2nav = qua2dcm(estimatedQuaternion);

        % Euler Angles
        estimatedRoll(i) = estimatedRoll(i)  - kf.xp(1);
        estimatedPitch(i) = estimatedPitch(i) - kf.xp(2);
        estimatedYaw(i) = estimatedYaw(i)   - kf.xp(3);

        % Velocity
        estimatedVelocities(i,1) = estimatedVelocities(i,1) - kf.xp(4);
        estimatedVelocities(i,2) = estimatedVelocities(i,2) - kf.xp(5);
        estimatedVelocities(i,3) = estimatedVelocities(i,3) - kf.xp(6);

        % Position
        estimatedLatitude(i) = estimatedLatitude(i) - kf.xp(7);
        estimatedLongitude(i) = estimatedLongitude(i) - kf.xp(8);
        estimatedAltitude(i) = estimatedAltitude(i)   - kf.xp(9);

        % Biases
        gyrDynamicBiases   = -kf.xp(10:12);
        accDynamicBiases   = -kf.xp(13:15);
        biases(measurementUpdateIdx,:) = [gyrDynamicBiases', accDynamicBiases'];

    end
end
%% Saving estimates to a single structure
estimates.t = imu.t(1:i, :);                                                % IMU time [s]
estimates.tg = gnss.t;                                                      % GPS time [s]
estimates.roll = estimatedRoll(1:i, :);                                            % Roll [rad]
estimates.pitch = estimatedPitch(1:i, :);                                          % Pitch [rad]
estimates.yaw = estimatedYaw(1:i, :);                                              % Yaw [rad]
estimates.vel = estimatedVelocities(1:i, :);                                              % Velo (NED) [m/s]
estimates.lat = estimatedLatitude(1:i, :);                                              % Lat [rad]
estimates.lon = estimatedLongitude(1:i, :);                                              % Lon [rad]
estimates.h = estimatedAltitude(1:i, :);                                                  % Alt [rad]
estimates.b = biases;                                                            % Biases [m/s; rad/s]

end
