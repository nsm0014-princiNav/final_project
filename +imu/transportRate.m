function omega_en_n = transportRate(lat, Vn, Ve, h)
% transport_rate: calculates the transport rate in the navigation frame.
%
% INPUT
%	lat, 1x1 latitude (rad).
%	Vn, 1x1 North velocity (m/s).
%   Ve, 1x1 East velocity (m/s).
%   h, altitude (m).
%
% OUTPUT
%	omega_en_n, 3x3 skew-symmetric transport rate matrix (rad/s).
%
% References:
%
%   Paul D. Groves. Principles of GNSS, Inertial, and
% Multisensor Integrated Navigation Systems. Second Edition.
% Eq. 5.44, page 177.

h = abs(h);

a = 6378137.0;                  % WGS84 Equatorial radius in meters
e = 0.0818191908425;            % WGS84 eccentricity

e2 = e^2;
den = 1 - e2 .* (sin(lat)).^2;

% Meridian radius of curvature: radius of curvature for North-South motion.
RM = a .* (1-e2) ./ (den).^(3/2);

% Normal radius of curvature: radius of curvature for East-West motion.
% AKA transverse radius.
RN = a ./ sqrt(den);

om_en_n(1,1) =   Ve / (RN + h);              % North
om_en_n(2,1) = -(Vn / (RM + h));             % East
om_en_n(3,1) = -(Ve * tan(lat) / (RN + h));  % Down

omega_en_n = formskewsym(om_en_n);

end
