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
gravity_nav2ecef = [gravity(estimatedLatitude(1), estimatedAltitude(1)); zeros(numIMUIterations, 3)];                                         

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
angularRate_Nav = DCMbody2nav*imu.wb(1,:)';

% Vector to update matrix F
upd = [gnss.vel(1,:) gnss.lat(1) gnss.h(1) specificForce_Nav' angularRate_Nav'];

% Update matrices F and G
[kf.F, kf.G] = F_update(upd, DCMbody2nav, imu);

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
kf = kf_update(kf);

for i = 2:numIMUIterations
    %% A Priori Estimates (Time Update)
    % Current time step
    timeStepIMU = imu.t(i) - imu.t(i-1);

    % Correcting IMU measurements with bias estimation
    wb_corrected = imu.wb(i,:)' - gyrDynamicBiases - imu.gb_sta';
    fb_corrected = imu.fb(i,:)' - accDynamicBiases - imu.ab_sta';

    % Converting initial body frame specific forces to nav frame
    specificForce_Nav = DCMbody2nav * fb_corrected;
    angularRate_Nav = DCMbody2nav * wb_corrected;

    % Velocity update
    vel = vel_update(specificForce_Nav, estimatedVelocities(i-1,:), omegaEci2Ecef_Nav, omegaEci2Navi_Nav, gravity_nav2ecef(i-1,:)', timeStepIMU);
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
    gravity_nav2ecef(i,:) = gravity(estimatedLatitude(i), estimatedAltitude(i));

    % Attitude update
    [estimatedQuaternion, DCMbody2nav, euler] = att_update(wb_corrected, DCMbody2nav, omegaEci2Ecef_Nav, omegaEci2Navi_Nav, timeStepIMU);
    estimatedRoll(i) = euler(1);
    estimatedPitch(i)= euler(2);
    estimatedYaw(i)  = euler(3);

    %% Only perform A Posteriori update if there are GPS Measurements at the current timestep
    measurementUpdateIdx = find(gnss.t >= (imu.t(i) - gnss.eps) & gnss.t < (imu.t(i) + gnss.eps));

    if ( ~isempty(measurementUpdateIdx) && measurementUpdateIdx > 1)

        % Meridian and normal radii of curvature update
        [RM,RN] = radius(estimatedLatitude(i));

        % Radians-to-meters matrix
        Tpr = diag([(RM + estimatedAltitude(i)), (RN + estimatedAltitude(i)) * cos(estimatedLatitude(i)), -1]);

        % Position innovations in meters
        zp = Tpr * ([estimatedLatitude(i); estimatedLongitude(i); estimatedAltitude(i);] - [gnss.lat(measurementUpdateIdx); gnss.lon(measurementUpdateIdx); gnss.h(measurementUpdateIdx);]);

        % Velocity innovations
        zv = (estimatedVelocities(i,:) - gnss.vel(measurementUpdateIdx,:))';

        %% KALMAN FILTER

        % Current GPS time step (time since last mesurement update)
        timeStepGPS = gnss.t(measurementUpdateIdx) - gnss.t(measurementUpdateIdx-1);

        % Vector to update matrix F
        upd = [estimatedVelocities(i,:) estimatedLatitude(i) estimatedAltitude(i) specificForce_Nav' angularRate_Nav'];

        % Matrices F and G update
        [kf.F, kf.G] = F_update(upd, DCMbody2nav, imu);

        % Matrix H update
        kf.H = [zeros(3),  eye(3) , zeros(3), zeros(3), zeros(3);
            zeros(3), zeros(3),   Tpr   , zeros(3), zeros(3)];
        kf.R = diag([gnss.stdv gnss.stdm]).^2;
        kf.z = [zv' zp']';

        % a posteriori states are forced to be zero (error-state approach)
        kf.xp = zeros(15 , 1);
        % Execution of the extended Kalman filter
        kf = kalman(kf, timeStepGPS);

        %% Correcting estimates with A Posteriori error state matrix xp
        % Quaternion
        qua_skew = -skewm(estimatedQuaternion(1:3));
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
