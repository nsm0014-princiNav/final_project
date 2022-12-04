%% Formatting
clc
clear
close all
format shortg

addpath imu\
addpath filter\
addpath convert\

%% Loading Truth Data, Simulated IMU Measurements, and Simulated GPS Measurements
load('data/ref.mat')
load('data/imu1.mat')
load('data/gnss.mat')

%% Running Loosely Coupled IMU/GPS Sensor Fusion Algorithm
stateEstimates = ins_gnss(imu1, gnss, 'dcm');

%% PLOTTING
plotData(stateEstimates,ref)

