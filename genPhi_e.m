function Phi_e = genPhi_e(C_b_e_est,pos_est,f_ib_b)

F = zeros(15);

omega_ie = [0; 0; 7.292115*10^(-5)]; % (rad/s) rotation rate of Earth
Omega_ie = formskewsym(omega_ie); % Skew of Earth Rotation

% First row
F(1:3,:) = [-Omega_ie, zeros(3), zeros(3), zeros(3), C_b_e_est];

% Second row
F_e_21 = -C_b_e_est*formskewsym(f_ib_b);

mu = 3.986004418e14; % Gravitational constant [m^3/s^2]
J2 = 1.082627e-3; % Second Gravitational constant [m^3/s^2]
R0 = 6378137; % WGS84 Equatorial radius [m]
gamma_est = (-mu/norm(pos_est)^3)*(pos_est + 1.5*J2*R0^2/norm(pos_est)^2*())
F_e_23 = (-2*gamma_est/r_e_es)*(pos_est'/norm(pos_est));
-tor_s * 2 * Gravity_ECEF(est_r_eb_e_old)/geocentric_radius * est_r_eb_e_old'/sqrt(est_r_eb_e_old' *est_r_eb_e_old);
F(4:6,:) = [F_e_21, -2*Omega_ie, F_e_23, C_b_e_est, zeros(3)];
% Third Row

% Fourth Row

% Fifth Row


end