function [IMU,GPS,REF] = orgData(imu,gps,tru,noise,ref_lla)
%% Organizing IMU data
IMU.t = imu(7,:);
IMU.fx = imu(1,:) + gaussianDistFCN([1 length(IMU.t)],noise.acc.drift(1),noise.acc.bias(1));
IMU.fy = imu(2,:) + gaussianDistFCN([1 length(IMU.t)],noise.acc.drift(2),noise.acc.bias(2));
IMU.fz = imu(3,:) + gaussianDistFCN([1 length(IMU.t)],noise.acc.drift(3),noise.acc.bias(3));
IMU.wx = imu(4,:) + gaussianDistFCN([1 length(IMU.t)],noise.gyr.drift(1),noise.gyr.bias(1));
IMU.wy = imu(5,:) + gaussianDistFCN([1 length(IMU.t)],noise.gyr.drift(2),noise.gyr.bias(2));
IMU.wz = imu(6,:) + gaussianDistFCN([1 length(IMU.t)],noise.gyr.drift(3),noise.gyr.bias(3));

%% Organizing GPS data
GPS.t = gps(7,:);
GPS.x = gps(1,:) + gaussianDistFCN([1 length(GPS.t)],noise.gps_pos.drift(1),0);
GPS.y = gps(2,:) + gaussianDistFCN([1 length(GPS.t)],noise.gps_pos.drift(2),0);
GPS.z = gps(3,:) + gaussianDistFCN([1 length(GPS.t)],noise.gps_pos.drift(3),0);
GPS.x_dot = gps(4,:) + gaussianDistFCN([1 length(GPS.t)],noise.gps_vel.drift(1),0);
GPS.y_dot = gps(5,:) + gaussianDistFCN([1 length(GPS.t)],noise.gps_vel.drift(2),0);
GPS.z_dot = gps(6,:) + gaussianDistFCN([1 length(GPS.t)],noise.gps_vel.drift(3),0);

%% Organizing Ref data
REF.t = tru(16,:);
REF.x = tru(1,:);
REF.y = tru(2,:);
REF.z = tru(3,:);
REF.x_dot = tru(4,:);
REF.y_dot = tru(5,:);
REF.z_dot = tru(6,:);
REF.roll = wrapTo180(tru(7,:));
REF.pitch = wrapTo180(tru(8,:));
REF.yaw = wrapTo180(tru(9,:));
REF.ax = tru(10,:);
REF.ay = tru(11,:);
REF.az = tru(12,:);
REF.wx = tru(13,:);
REF.wy = tru(14,:);
REF.wz = tru(15,:);
REF.ref_lla = ref_lla;
REF.LLA = ecef2lla([REF.x;REF.y;REF.z]'); 

end

