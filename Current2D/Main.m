yalmip clear
clear all
close all

%% Initialization
horizon = 120;
T = 0.5;
% initial u = 150 m/s, height = 3000m
x0 = [150,0,0, 0, 0, 3000, 0, 0]';
% final u = 70 m/s, distance = 12000m
xT = [70, 0, 0, 0, 12000, 0, 0, 0]';

%% Reference
[feas_R, xOpt_R, uOpt_R, JOpt_R] = CFTOC_2D_Reference(x0, horizon, T, xT);
plot(xOpt_R(5,:),xOpt_R(6,:))
save('Reference.mat','xOpt_R')

%% MPC Tracking
step = 10;
load('Reference.mat')
[xOpt, uOpt, feas] = Plane_MPC(step, horizon, T, x0, xT, xOpt_R);
fig1 = figure(1);
plot(xOpt(5,:),xOpt(6,:),'r*')
hold on
plot(xOpt_R(5,:),xOpt_R(6,:))

legend('MPC with disturbance','reference')
xlabel('distance [m]')
ylabel('height [m]')
title('MPC tracking with 5% disturbances in relation with vertical velocity')

save('Plotting.mat', 'xOpt_R', 'xOpt')

%% Plot Plane
load('Plotting.mat')
airplane_plot(xOpt_R,xOpt)


%% Attribution
% ME C231A, UC Berkeley, Fall 2017
% Final Project
% MPC tracking: Optimization of Fuel Efficiency during Landing of Commercial Airplane with Disturbances
%
% Group members: 
% Minglong Li
% Fu-Hsiang Chang
% Chia Shen Shyu
% Rex Chen



