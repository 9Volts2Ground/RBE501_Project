clc; clear all; close all; 

addpath('..')

drone_param;

pos = [10,0,5];
orientation = [pi/6,pi/8,0];
time = 0;

%Pack data into array
u(1:3) = pos;
u(4:6) = orientation;
u(7) = time;

%Test plotter
drawDrone(u,P)