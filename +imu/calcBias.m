function [IMUbiases,GPSbiases] = calcBias(staticData)
                                              
IMUbiases.arw = mean(diff(staticData.memsense.imu.angularVelocity',1,1));      % arw: 1x3 angle random walks (rad/s/root-Hz)
IMUbiases.vrw = mean(diff(staticData.memsense.imu.linearAcceleration',1,1));   % vrw: 1x3 velocity random walks (m/s^2/root-Hz)
IMUbiases.g_std = std(staticData.memsense.imu.angularVelocity');               % g_std: 1x3 gyros standard deviations (radians/s)
IMUbiases.a_std = std(staticData.memsense.imu.linearAcceleration');            % a_std: 1x3 accrs standard deviations (m/s^2)
IMUbiases.gb_sta = mean(staticData.memsense.imu.angularVelocity');             % gb_sta: 1x3 gyros static biases or turn-on biases (radians/s)
IMUbiases.ab_sta = mean(staticData.memsense.imu.linearAcceleration');          % ab_sta: 1x3 accrs static biases or turn-on biases (m/s^2)
IMUbiases.gb_corr = [30 30 30];                                                % gb_corr: 1x3 gyros correlation times (seconds)
IMUbiases.ab_corr = [30 30 30];                                                % ab_corr: 1x3 accrs correlation times (seconds)
IMUbiases.gb_dyn = -IMUbiases.gb_sta./IMUbiases.gb_corr;                        % gb_dyn: 1x3 gyros dynamic biases or bias instabilities (radians/s)
IMUbiases.ab_dyn = -IMUbiases.ab_sta./IMUbiases.ab_corr;                        % ab_dyn: 1x3 accrs dynamic biases or bias instabilities (m/s^2)
IMUbiases.gb_psd = ones(1,3)*deg2rad(0.5);                                     % gb_psd: 1x3 gyros dynamic biases root-PSD (rad/s/root-Hz)
IMUbiases.ab_psd = ones(1,3)*0.004903325;                                      % ab_psd: 1x3 accrs dynamic biases root-PSD (m/s^2/root-Hz)


lat_rad = deg2rad(staticData.novatel_local.odom.positionLla(1,:));
long_rad = deg2rad(staticData.novatel_local.odom.positionLla(2,:));
h = staticData.novatel_local.odom.positionLla(3,:);

GPSbiases.std = std([lat_rad;long_rad;h]');                                    % std: 1x3 position standard deviations (rad, rad, m)
GPSbiases.stdm = std(staticData.novatel_local.odom.positionEcef');             % stdm: 1x3 position standard deviations (m, m, m)
GPSbiases.stdv = std(staticData.novatel_local.odom.velocityEcef');             % stdv: 1x3 velocity standard deviations (m/s)                                                            

end