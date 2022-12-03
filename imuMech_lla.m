function [C_b2n_plus,v_eb2n_plus,lat_plus,long_plus,alt_plus,euler] = imuMech_lla(omega_ib_b,f_ib_b,C_b2n_minus,v_eb2n_minus,lat_minus,long_minus,alt_minus,dt)

% Attitude Update
Omega_ib_b = formskewsym(omega_ib_b);
omega_ie = 7.292115e-5;
Omega_ie_n_minus = [     0,         sin(lat_minus),           0         ; ...
                    -sin(lat_minus),    0         ,      -cos(lat_minus); ...
                         0,         cos(lat_minus),           0         ].*omega_ie;
[RN,RE] = radii(lat_minus);

omega_en_n_minus = [v_eb2n_minus(2)/(RE + alt_minus); ...
                   -v_eb2n_minus(1)/(RN + alt_minus); ...
                   (-v_eb2n_minus(2)*tan(lat_minus))/(RE + alt_minus)];
Omega_en_n_minus = formskewsym(omega_en_n_minus);

[C_b2n_plus, euler] = att_update(omega_ib_b, C_b2n_minus, Omega_ie_n_minus, Omega_en_n_minus, dt);
% euler(1) = 0; % No Roll
% Velocity Update
gravity_b_n = g_b_n(lat_minus,alt_minus);
coriolis = (Omega_en_n_minus + 2*Omega_ie_n_minus);
force_corrected = (f_ib_b - gravity_b_n' - coriolis*v_eb2n_minus);
v_eb2n_plus = v_eb2n_minus + force_corrected*dt;

% Position Update
alt_plus = alt_minus - (dt/2)*(v_eb2n_minus(3) + v_eb2n_plus(3));

num1 = v_eb2n_minus(1);
den1 = RN + alt_minus;
num2 = v_eb2n_plus(1);
den2 = RN + alt_plus;

lat_plus = lat_minus + (dt/2)*(num1/den1 + num2/den2);

num1 = v_eb2n_minus(2);
den1 = (RE + alt_minus)*cos(lat_minus);

[~,RE] = radii(lat_plus);

num2 = v_eb2n_plus(2);
den2 = (RE + alt_plus)*cos(lat_plus);

long_plus = long_minus + (dt/2)*(num1/den1 + num2/den2);
end