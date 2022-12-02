function [C_b_e_new,vel_new,pos_new,imuBias_new,pMat_new] = KF(GPS_pos,GPS_vel,dtGPS,C_b_e_old,vel_old,pos_old,imuBias_old,pMat_old,f_ib_b,lat_old)

% Constants
c = 299792458; % Speed of light in m/s
omega_ie = 7.292115E-5;  % Earth rotation rate in rad/s
R_0 = 6378137; % WGS84 Equatorial radius in meters
e = 0.0818191908425; % WGS84 eccentricity

Omega_ie = formskewsym([0 0 omega_ie]);

geocentric_radius = R_0 / sqrt(1 - (e * sind(lat_old))^2) *...
    sqrt(cosd(lat_old)^2 + (1 - e^2)^2 * sind(lat_old)^2); % from (2.137)

Phi = eye(15);
Phi(1:3,1:3) = Phi(1:3,1:3) - Omega_ie*dtGPS;
Phi(1:3,13:15) = C_b_e_old*dtGPS;
Phi(4:6,1:3) = -dtGPS*formskewsym(C_b_e_old*f_ib_b');
Phi(4:6,4:6) = Phi(4:6,4:6) - 2*Omega_ie*dtGPS;
Phi(4:6,7:9) = -dtGPS*2*GravModel_ECEF(pos_old)/geocentric_radius*pos_old'/sqrt(pos_old'*pos_old);
Phi(4:6,10:12) = C_b_e_old*dtGPS;
Phi(7:9,4:6) = eye(3)*dtGPS;

Q = zeros(15);
Q(1:3,1:3) = eye(3)*0*dtGPS;
Q(4:6,4:6) = eye(3)*0*dtGPS;
Q(10:12,10:12) = eye(3)*0*dtGPS;
Q(13:15,13:15) = eye(3)*0*dtGPS;

x_est_propagated(1:15,1) = 0;

P_matrix_propagated = Phi*(pMat_old + 0.5* Q)*Phi' + 0.5*Q;

H = zeros(6,15);
H(1:3,7:9) = -eye(3);
H(4:6,4:6) = -eye(3);

R(1:3,1:3) = eye(3) *0^2;
R(1:3,4:6) = zeros(3);
R(4:6,1:3) = zeros(3);
R(4:6,4:6) = eye(3) *0^2;

K = P_matrix_propagated * H'*inv(H*P_matrix_propagated*H' + R);

delta_z(1:3,1) = GPS_pos' - pos_old;
delta_z(4:6,1) = GPS_vel' - vel_old;

x_est_new = x_est_propagated + K * delta_z;

pMat_new = (eye(15) - K * H) * P_matrix_propagated;

% Correct attitude, velocity, and position using (14.7-9)
C_b_e_new = (eye(3) - formskewsym(x_est_new(1:3))) * C_b_e_old;
vel_new = vel_old - x_est_new(4:6);
pos_new = pos_old - x_est_new(7:9);

% Update IMU bias estimates
imuBias_new = imuBias_old + x_est_new(10:15);

end