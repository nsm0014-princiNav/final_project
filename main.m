%% Formatting
clc
clear
close all
format shortg
addpath("data\")
%% Loading in .mat files from Truck data
load("random.mat","random")
load("static.mat","static")
load("turn.mat","turn")

for i = length(turn.IMU.Time)

    [p_eb_e, v_eb_e, C_b2e] = Mechanize_ECEF(f_ib_b, omega_ib_b, dt, old_PVA);
    
    old_PVA.pos = 1;
    old_PVA.vel = 1;
    old_PVA.C_b2e = 1;
end

