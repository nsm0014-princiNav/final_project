%% Formatting
clc
clear
close all
format shortg
addpath("data\")
%% Loading in .mat files from Truck data
load("random_raw.mat","random_data")
load("static_raw.mat","static_data")
load("turn_raw.mat","turn_data")

%% Wheel Speed
Measurement = [1:length(random_data.j1939.wheel_speed.time)]';
Time = random_data.j1939.wheel_speed.time';
Front_Left_Wheel = random_data.j1939.wheel_speed.frontLeft';
Front_Right_Wheel = random_data.j1939.wheel_speed.frontRight';

random.wheelSpeed = table(Measurement,Time,Front_Left_Wheel,Front_Right_Wheel);

Measurement = [1:length(static_data.j1939.wheel_speed.time)]';
Time = static_data.j1939.wheel_speed.time';
Front_Left_Wheel = static_data.j1939.wheel_speed.frontLeft';
Front_Right_Wheel = static_data.j1939.wheel_speed.frontRight';

static.wheelSpeed = table(Measurement,Time,Front_Left_Wheel,Front_Right_Wheel);

Measurement = [1:length(turn_data.j1939.wheel_speed.time)]';
Time = turn_data.j1939.wheel_speed.time';
Front_Left_Wheel = turn_data.j1939.wheel_speed.frontLeft';
Front_Right_Wheel = turn_data.j1939.wheel_speed.frontRight';

turn.wheelSpeed = table(Measurement,Time,Front_Left_Wheel,Front_Right_Wheel);

%% Steer Angle
Measurement = [1:length(random_data.j1939.steer_angle.time)]';
Time = random_data.j1939.steer_angle.time';
Steer_Angle = random_data.j1939.steer_angle.steerAngle';

random.steerAngle = table(Measurement,Time,Steer_Angle);

Measurement = [1:length(static_data.j1939.steer_angle.time)]';
Time = static_data.j1939.steer_angle.time';
Steer_Angle = static_data.j1939.steer_angle.steerAngle';

static.steerAngle = table(Measurement,Time,Steer_Angle);

Measurement = [1:length(turn_data.j1939.steer_angle.time)]';
Time = turn_data.j1939.steer_angle.time';
Steer_Angle = turn_data.j1939.steer_angle.steerAngle';

turn.steerAngle = table(Measurement,Time,Steer_Angle);

%% IMU
Measurement = [1:length(random_data.memsense.imu.time)]';
Time = random_data.memsense.imu.time';
Linear_Acceleration = random_data.memsense.imu.linearAcceleration';
Angular_Velocity = random_data.memsense.imu.angularVelocity';

random.IMU = table(Measurement,Time,Linear_Acceleration,Angular_Velocity);

Measurement = [1:length(static_data.memsense.imu.time)]';
Time = static_data.memsense.imu.time';
Linear_Acceleration = static_data.memsense.imu.linearAcceleration';
Angular_Velocity = static_data.memsense.imu.angularVelocity';

static.IMU = table(Measurement,Time,Linear_Acceleration,Angular_Velocity);

Measurement = [1:length(turn_data.memsense.imu.time)]';
Time = turn_data.memsense.imu.time';
Linear_Acceleration = turn_data.memsense.imu.linearAcceleration';
Angular_Velocity = turn_data.memsense.imu.angularVelocity';

turn.IMU = table(Measurement,Time,Linear_Acceleration,Angular_Velocity);

%% GPS
Measurement = [1:length(random_data.novatel_local.odom.time)]';
Time = random_data.novatel_local.odom.time';
Position = random_data.novatel_local.odom.positionEcef';
Velocity = random_data.novatel_local.odom.velocityEcef';

random.GPS = table(Measurement,Time,Position,Velocity);

Measurement = [1:length(static_data.novatel_local.odom.time)]';
Time = static_data.novatel_local.odom.time';
Position = static_data.novatel_local.odom.positionEcef';
Velocity = static_data.novatel_local.odom.velocityEcef';

static.GPS = table(Measurement,Time,Position,Velocity);

Measurement = [1:length(turn_data.novatel_local.odom.time)]';
Time = turn_data.novatel_local.odom.time';
Position = turn_data.novatel_local.odom.positionEcef';
Velocity = turn_data.novatel_local.odom.velocityEcef';

turn.GPS = table(Measurement,Time,Position,Velocity);
