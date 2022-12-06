%% Formatting
clc
clear
close all
format shortg

%% Loading Truth Data, Simulated IMU Measurements, and Simulated GPS Measurements
load('data/ref.mat')
load('data/simulatedIMU.mat')
load('data/simulatedGPS.mat')

%% Running Loosely Coupled IMU/GPS Sensor Fusion Algorithm
%% Pre-allocating arrays for speed
% Length of time
numIMUIterations = length(simulatedIMU.t);
numGPSIterations = length(simulatedGPS.t);

% Attitude
estimatedRoll = [simulatedIMU.ini_align(1); zeros(numIMUIterations-1, 1)];
estimatedPitch = [simulatedIMU.ini_align(2); zeros(numIMUIterations-1, 1)];
estimatedYaw = [simulatedIMU.ini_align(3); zeros(numIMUIterations-1, 1)];

% Velocity
estimatedVelocities = zeros(numIMUIterations, 3);

% Position
estimatedLatitude = [simulatedGPS.lat(1); zeros(numIMUIterations-1, 1)];
estimatedLongitude = [simulatedGPS.lon(1); zeros(numIMUIterations-1, 1)];
estimatedAltitude = [simulatedGPS.h(1); zeros(numIMUIterations-1, 1)];

% Gravity
estimatedGravity_Nav = [gravity(estimatedLatitude(1), estimatedAltitude(1)); zeros(numIMUIterations, 3)];

% Bias
gyrDynamicBiases = simulatedIMU.gb_dyn';
accDynamicBiases = simulatedIMU.ab_dyn';
biases = [gyrDynamicBiases', accDynamicBiases'; zeros(numGPSIterations-1, 6)];

% Initial Rotation
DCMnav2body = genDCM('rad',[estimatedYaw(1) estimatedPitch(1) estimatedRoll(1)],[3 2 1]);
DCMbody2nav = DCMnav2body';

% Earth's Rotation Rate
omegaEci2Ecef_Nav = (7.2921155e-5) .* [0          sin(estimatedLatitude(1))  0; ...
    -sin(estimatedLatitude(1))  0        -cos(estimatedLatitude(1)); ...
    0          cos(estimatedLatitude(1))  0;];

den = 1 - 0.0818191908425^2 .* (sin(estimatedLatitude(1))).^2;
RM = 6378137.0 .* (1-0.0818191908425^2) ./ (den).^(3/2);
RN = 6378137.0 ./ sqrt(den);

om_en_n(1,1) =   estimatedVelocities(1,2) / (RN + estimatedAltitude(1));              % North
om_en_n(2,1) = -(estimatedVelocities(1,1) / (RM + estimatedAltitude(1)));             % East
om_en_n(3,1) = -(estimatedVelocities(1,2) * tan(estimatedLatitude(1)) / (RN + estimatedAltitude(1)));  % Down

omegaEci2Navi_Nav = formskewsym(om_en_n);

% A Priori Error State
errorState_minus = [zeros(9,1); simulatedIMU.gb_dyn'; simulatedIMU.ab_dyn'];

% A Priori Covariance Matrix
pMinus = diag([simulatedIMU.ini_align_err, simulatedGPS.stdv, simulatedGPS.std, simulatedIMU.gb_dyn, simulatedIMU.ab_dyn].^2);

% IMU Processing Noise Matrix
procNoiseMat  = diag([simulatedIMU.arw, simulatedIMU.vrw, simulatedIMU.gb_psd, simulatedIMU.ab_psd].^2);

% Converting initial body frame specific forces to nav frame
specificForce_Nav = DCMbody2nav*simulatedIMU.fb(1,:)';

% Update matrices F and G
[dynamicModelMat, noiseDistMat] = dynamicModel(0,0,0, estimatedLatitude(1), estimatedAltitude(1),specificForce_Nav', DCMbody2nav,RM,RN);

rad2range = diag([(RM + simulatedGPS.h(1)), (RN + simulatedGPS.h(1))*cos(simulatedGPS.lat(1)), -1]);  % radians-to-meters

% Measurement Matrix
measurementMat = [zeros(3),  eye(3) , zeros(3), zeros(3), zeros(3);
    zeros(3), zeros(3),   rad2range   , zeros(3), zeros(3)];

% GPS Measurement Noise Matrix
measNoiseMat = diag([simulatedGPS.stdv simulatedGPS.stdm]).^2;

% Innovation Matrix
deltaZ = zeros(6,1);

% Propagate prior estimates to get xp(1) and Pp(1)
kalmanGain = (pMinus*measurementMat')*(measurementMat*pMinus*measurementMat' + measNoiseMat)^(-1) ;			% Kalman gain matrix

% Step 4, update the a posteriori state vector, xp
errorState_plus = errorState_minus + kalmanGain*deltaZ;

% Step 5, update the a posteriori covariance matrix, Pp
pPlus = pMinus - kalmanGain*(measNoiseMat + measurementMat * pMinus * measurementMat')*kalmanGain';

for i = 2:numIMUIterations
    %% A Priori Estimates (Time Update)
    % Current time step
    timeStepIMU = simulatedIMU.t(i) - simulatedIMU.t(i-1);

    % Correcting IMU measurements with bias estimationp
    wb_corrected = simulatedIMU.wb(i,:)' - gyrDynamicBiases - simulatedIMU.gb_sta';
    fb_corrected = simulatedIMU.fb(i,:)' - accDynamicBiases - simulatedIMU.ab_sta';

    % Converting initial body frame specific forces to nav frame
    specificForce_Nav = DCMbody2nav*fb_corrected;

    % Velocity update
    estimatedVelocities(i,:) = estimatedVelocities(i-1,:) + ((specificForce_Nav + estimatedGravity_Nav(i-1,:)' - ((omegaEci2Navi_Nav + 2*omegaEci2Ecef_Nav)*estimatedVelocities(i-1,:)'))'*timeStepIMU);

    % Position update
    pos = position([estimatedLatitude(i-1) estimatedLongitude(i-1) estimatedAltitude(i-1)], estimatedVelocities(i,:), timeStepIMU);
    estimatedLatitude(i) = pos(1);
    estimatedLongitude(i) = pos(2);
    estimatedAltitude(i) = pos(3);

    % Turn-rates update with both updated velocity and position
    omegaEci2Ecef_Nav = (7.2921155e-5).*[0          sin(estimatedLatitude(i))  0; ...
        -sin(estimatedLatitude(i))  0        -cos(estimatedLatitude(i)); ...
        0          cos(estimatedLatitude(i))  0];
    estimatedAltitude(i) = abs(estimatedAltitude(i));

    den = 1 - 0.0818191908425^2 .* (sin(estimatedLatitude(i))).^2;
    RM = 6378137.0 .* (1-0.0818191908425^2) ./ (den).^(3/2);
    RN = 6378137.0 ./ sqrt(den);

    om_en_n(1,1) =   estimatedVelocities(i,2)/(RN + estimatedAltitude(i));
    om_en_n(2,1) = -(estimatedVelocities(i,1)/(RM + estimatedAltitude(i)));
    om_en_n(3,1) = -(estimatedVelocities(i,2)*tan(estimatedLatitude(i))/(RN + estimatedAltitude(i)));

    omegaEci2Navi_Nav = formskewsym(om_en_n);

    % Gravity update
    estimatedGravity_Nav(i,:) = gravity(estimatedLatitude(i), estimatedAltitude(i));

    % Attitude update
    [estimatedQuaternion, DCMbody2nav, euler] = attitude(wb_corrected, DCMbody2nav, omegaEci2Ecef_Nav, omegaEci2Navi_Nav, timeStepIMU);
    estimatedRoll(i) = euler(1);
    estimatedPitch(i) = euler(2);
    estimatedYaw(i)  = euler(3);

    %% Only perform A Posteriori update if there are GPS Measurements at the current timestep
    measurementUpdateIdx = find(simulatedGPS.t >= (simulatedIMU.t(i) - simulatedGPS.eps) & simulatedGPS.t < (simulatedIMU.t(i) + simulatedGPS.eps));

    if ( ~isempty(measurementUpdateIdx) && measurementUpdateIdx > 1)
        % Current GPS time step (time since last mesurement update)
        timeStepGPS = simulatedGPS.t(measurementUpdateIdx) - simulatedGPS.t(measurementUpdateIdx-1);

        % Grouping current estimated position
        estimatedLLA = [estimatedLatitude(i); estimatedLongitude(i); estimatedAltitude(i)];
        measurementLLA = [simulatedGPS.lat(measurementUpdateIdx); simulatedGPS.lon(measurementUpdateIdx); simulatedGPS.h(measurementUpdateIdx)];

        % Meridian and normal radii of curvature update
        den = 1 - 0.0818191908425^2 .* (sin(estimatedLatitude(i))).^2;
        RM = 6378137.0 .* (1-0.0818191908425^2) ./ (den).^(3/2);
        RN = 6378137.0 ./ sqrt(den);

        % Radians-to-meters matrix
        rad2range = diag([(RM + estimatedAltitude(i)), (RN + estimatedAltitude(i)) * cos(estimatedLatitude(i)), -1]);

        % Innovation Matrix
        deltaZ = [(estimatedVelocities(i,:) - simulatedGPS.vel(measurementUpdateIdx,:))'; ...
            rad2range*(estimatedLLA - measurementLLA)];

        % Dynamic Model (F) and Noise Distribution Matrix (G)
        [dynamicModelMat, noiseDistMat] = dynamicModel(estimatedVelocities(i,1), estimatedVelocities(i,2), estimatedVelocities(i,3), estimatedLatitude(i), estimatedAltitude(i), specificForce_Nav', DCMbody2nav,RM,RN);

        % Matrix H update
        measurementMat = [zeros(3),  eye(3) , zeros(3), zeros(3), zeros(3);
            zeros(3), zeros(3),   rad2range   , zeros(3), zeros(3)];

        % GPS Measurement Noise Matrix
        measNoiseMat = diag([simulatedGPS.stdv simulatedGPS.stdm]).^2;

        % Dynamic Model in Discrete
        discreteDynamicModelMat =  expm(dynamicModelMat * timeStepGPS);

        % IMU Processing Noise Matrix
        discreteProcNoiseMat = (noiseDistMat * procNoiseMat * noiseDistMat') .* timeStepGPS;

        % A Priori Error State Matrix
        errorState_minus = zeros(15, 1);

        % A Priori Covariance Matrix
        pMinus = (discreteDynamicModelMat * pPlus * discreteDynamicModelMat') + discreteProcNoiseMat;

        % Kalman Gain
        kalmanGain = (pMinus * measurementMat') * (measNoiseMat + measurementMat * pMinus * measurementMat')^(-1) ;			% Kalman gain matrix

        % A Postierori Error State Vector
        errorState_plus = kalmanGain * deltaZ;

        % A Postierori Covariance Matrix
        pPlus = pMinus - kalmanGain * (measNoiseMat + measurementMat * pMinus * measurementMat') * kalmanGain';

        %% Correcting estimates with A Posteriori error state matrix xp
        % Quaternion
        quaternion_skew = -formskewsym(estimatedQuaternion(1:3));
        quaternionTensor = [estimatedQuaternion(4)*eye(3) + quaternion_skew; -estimatedQuaternion(1:3)'];
        estimatedQuaternion = estimatedQuaternion + 0.5 .* quaternionTensor * errorState_plus(1:3);
        estimatedQuaternion = estimatedQuaternion / norm(estimatedQuaternion);

        % DCM
        DCMbody2nav = qua2dcm(estimatedQuaternion);

        % Euler Angles
        estimatedRoll(i) = estimatedRoll(i)  - errorState_plus(1);
        estimatedPitch(i) = estimatedPitch(i) - errorState_plus(2);
        estimatedYaw(i) = estimatedYaw(i)   - errorState_plus(3);

        % Velocity
        estimatedVelocities(i,1) = estimatedVelocities(i,1) - errorState_plus(4);
        estimatedVelocities(i,2) = estimatedVelocities(i,2) - errorState_plus(5);
        estimatedVelocities(i,3) = estimatedVelocities(i,3) - errorState_plus(6);

        % Position
        estimatedLatitude(i) = estimatedLatitude(i) - errorState_plus(7);
        estimatedLongitude(i) = estimatedLongitude(i) - errorState_plus(8);
        estimatedAltitude(i) = estimatedAltitude(i)   - errorState_plus(9);

        % Biases
        gyrDynamicBiases   = -errorState_plus(10:12);
        accDynamicBiases   = -errorState_plus(13:15);
        biases(measurementUpdateIdx,:) = [gyrDynamicBiases', accDynamicBiases'];

    end
end
%% Saving estimates to a single structure
stateEstimates.t = simulatedIMU.t(1:i, :);                                                % IMU time [s]
stateEstimates.tg = simulatedGPS.t;                                                      % GPS time [s]
stateEstimates.roll = estimatedRoll(1:i, :);                                            % Roll [rad]
stateEstimates.pitch = estimatedPitch(1:i, :);                                          % Pitch [rad]
stateEstimates.yaw = estimatedYaw(1:i, :);                                              % Yaw [rad]
stateEstimates.vel = estimatedVelocities(1:i, :);                                              % Velo (NED) [m/s]
stateEstimates.lat = estimatedLatitude(1:i, :);                                              % Lat [rad]
stateEstimates.lon = estimatedLongitude(1:i, :);                                              % Lon [rad]
stateEstimates.h = estimatedAltitude(1:i, :);                                                  % Alt [rad]
stateEstimates.b = biases;                                                            % Biases [m/s; rad/s]


%% PLOTTING
plotData(stateEstimates,ref)

