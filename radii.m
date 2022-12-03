function [RN,RE] = radii(lat)

R0 = 6378137.0; % Earth's Radius [m]
e = 0.0818191908425; % WGS84 eccentricity

num = R0*(1 - e^2);
den = (1 - e^2*(sin(lat)^2))^(3/2);
RN = num/den;

num = R0;
den = sqrt(1 - e^2*(sin(lat)^2));

RE = num/den;
end