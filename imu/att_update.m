function [qua, DCMbn, euler] = att_update(wb, DCMbn, omega_ie_n, omega_en_n, dt)



%% Gyros output correction for Earth and transport rates

om_ie_n = skewm_inv(omega_ie_n);
om_en_n = skewm_inv(omega_en_n);
wb = (wb - DCMbn' * (om_ie_n + om_en_n));  % Titterton, Eq. 3.29, p. 32


delta_theta = wb * dt;                  % Incremental Euler angles
DCMbn = dcm_update(DCMbn, delta_theta); % DCM update
euler = dcm2euler(DCMbn);               % Euler angles update
qua   = euler2qua(euler);               % Quaternion update
qua   = qua / norm(qua);                % Brute-force normalization

end
