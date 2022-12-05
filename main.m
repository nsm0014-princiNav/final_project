%% Formatting
clc
clear
close all
format shortg

addpath imu\
addpath convert\

%% Loading Truth Data, Simulated IMU Measurements, and Simulated GPS Measurements
load('data/ref.mat')
load('data/simulatedIMU.mat')
load('data/simulatedGPS.mat')

%% Running Loosely Coupled IMU/GPS Sensor Fusion Algorithm
stateEstimates = LC_EKF(simulatedIMU, simulatedGPS);

%% PLOTTING
plotData(stateEstimates,ref)

