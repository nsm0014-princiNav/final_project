function [est] = LC(IMU,GPS,REF)

%% Initialization

pos_est = [GPS.x(1);GPS.y(1);GPS.z(1)];
vel_est = zeros(3,1);
C_b_e_est = genDCM('deg',[0 0 0],[3 2 1]);
lla = REF.ref_lla;
lat_old = lla(1);

oldGPSTime = 0;
idxGPS = 1;
imuBias_old = zeros(1,6);
imuBias_est = zeros(1,6);
pMat_old = eye(15);
% pMat_old(1:3,1:3) = eye(3)*0.125^2;
% pMat_old(4:6,4:6) = eye(3)*0.25^2;
% pMat_old(7:9,7:9) = eye(3)*0.5^2;
% pMat_old(10:12,10:12) = eye(3)*0.05^2;
% pMat_old(13:15,13:15) = eye(3)*0.0005^2;

pMat_est = pMat_old;

est.pos(1:3,:) = [pos_est zeros(3,length(IMU.t)-1)];
est.pos_lla(1:3,:) = [REF.ref_lla zeros(3,length(IMU.t)-1)];
est.vel(1:3,:) = [vel_est zeros(3,length(IMU.t)-1)];
%% Start Kalman Filter
for i = 2:length(IMU.t)

    % Calculating current time step
    dt = IMU.t(i) - IMU.t(i-1);

    % Mechanizing IMU
    f_ib_b = [IMU.fx(i) IMU.fy(i) IMU.fz(i)];
    omega_ib_b = [IMU.wx(i) IMU.wy(i) IMU.wz(i)];

    [pos_est,vel_est,C_b_e_est] = Mechanize_ECEF(f_ib_b, omega_ib_b, dt, pos_est,vel_est,C_b_e_est);

    if IMU.t(i) - oldGPSTime >= 0.1

        idxGPS = idxGPS + 1;
        dtGPS = IMU.t(i) - oldGPSTime;
        oldGPSTime = IMU.t(i);
        GPS.t(idxGPS)

        pos_est = [GPS.x(idxGPS);GPS.y(idxGPS);GPS.z(idxGPS)];
        vel_est = [GPS.x_dot(idxGPS);GPS.y_dot(idxGPS);GPS.z_dot(idxGPS)];
        C_b_e_est = genDCM('deg',[REF.roll(i) -REF.pitch(i) REF.yaw(i)],[1 2 3]);

%         [C_b_e_est,vel_est,pos_est,imuBias_est,pMat_est] = ...
%             KF([GPS.x(idxGPS) GPS.y(idxGPS) GPS.z(idxGPS)], ...
%             [GPS.x_dot(idxGPS) GPS.y_dot(idxGPS) GPS.z_dot(idxGPS)], ...
%             dtGPS,...
%             C_b_e_est, ...
%             vel_est,...
%             pos_est, ...
%             imuBias_est,...
%             pMat_est,...
%             f_ib_b,...
%             lat_old);


    end

%     lla = ecef2lla(pos_est');
%     lat_old = lla(1);

%% Saving and Exporting Data Structure
est.pos(1:3,i) = pos_est;
est.pos_lla(1:3,i) = ecef2lla(pos_est');
est.vel(1:3,i) = vel_est;
est.eul(1:3,i) = [atan2d(C_b_e_est(2,3),C_b_e_est(3,3)),...
                  asind(C_b_e_est(1,3)),...
                  atan2d(C_b_e_est(1,2),C_b_e_est(1,1))];
end

end