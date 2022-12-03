function [est] = LC(IMU,GPS,REF)

%% Initialization

lat_minus = REF.ref_lla(1)*(pi/180);
long_minus = REF.ref_lla(2)*(pi/180);
alt_minus = REF.ref_lla(3);
v_eb2n_minus = zeros(3,1);
C_b2n_minus = [0 1 0;1 0 0;0 0 -1];

oldGPSTime = 0;
idxGPS = 1;

est.pos(1:3,1) = [lat_minus;long_minus;alt_minus];
%% Start Kalman Filter
for i = 2:length(IMU.t)

    % Calculating current time step
    dt = IMU.t(i) - IMU.t(i-1);

    % Mechanizing IMU
    f_ib_b = [0 -1 0;1 0 0;0 0 -1]*[IMU.fx(i) IMU.fy(i) IMU.fz(i)]';
    omega_ib_b = [0 -1 0;1 0 0;0 0 -1]*deg2rad([IMU.wx(i) IMU.wy(i) IMU.wz(i)]');

    [C_b2n_plus,v_eb2n_plus,lat_plus,long_plus,alt_plus,euler] = imuMech_lla(omega_ib_b,f_ib_b,C_b2n_minus,v_eb2n_minus,lat_minus,long_minus,alt_minus,dt);

    % Only perform measurement update if a GPS measurement is available
    if IMU.t(i) - oldGPSTime >= 0.1

        % Getting time steps
        idxGPS = idxGPS + 1;
        dtGPS = IMU.t(i) - oldGPSTime;
        oldGPSTime = IMU.t(i);

        % Generating system matrix
%         Phi_e = genPhi_e(C_b_e_est,pos_est,f_ib_b)





    end
% Updating Values
C_b2n_minus = C_b2n_plus;
v_eb2n_minus = v_eb2n_plus;
lat_minus = lat_plus;
long_minus =long_plus;
alt_minus = alt_plus;


%     lla = ecef2lla(pos_est');
%     lat_old = lla(1);

%% Saving and Exporting Data Structure
est.pos(1:3,i) = [lat_plus;long_plus;alt_plus];
est.vel(1:3,i) = v_eb2n_plus;
est.eul(1:3,i) = rad2deg(euler);
end

end