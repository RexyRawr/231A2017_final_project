%% ME C231A, EECS C220B, Final Project
yalmip clear
clear all
close all

M = 20;
step = 70; %N
T = 1;
xT = [10000, 5000, 0];
x0 = [200, 0, 0, 0, 0, 0, 0, 0, 5000, 0, 0, 0]';


% [feas, xOpt, uOpt, JOpt] = solveOPTOriginal(x0, step, T, xT);
[feas, xOpt, uOpt, predErr] = MPC3D(x0, M, step, xT, T)


% [feas, xOpt, uOpt, predErr] = MPC(x0, M, step, xT, T)

plot3(xOpt(7,:),xOpt(8,:),xOpt(9,:))
grid on


%%
% %        
%         z(12, i) >= -5 * pi / 180,...
%         z(12, i) <= 5 * pi / 180,...
%         z(7, i+1) == z(7, i) + cos(z(10, i))*cos(z(12, i))*T * z(1, i+1) + (-cos(z(11, i))*sin(z(12, i))+sin(z(11, i))*sin(z(10, i))*cos(z(12, i)))*T * z(2, i+1) + (sin(z(11, i))*sin(z(12, i))+cos(z(11, i))*sin(z(10, i))*cos(z(12, i)))*T * z(3, i+1),...
%         z(8, i+1) == z(8, i) + cos(z(10, i))*sin(z(12, i))*T * z(1, i+1) + (cos(z(11, i))*cos(z(12, i))+sin(z(11, i))*sin(z(10, i))*sin(z(12, i)))*T * z(2, i+1) + (-sin(z(11, i))*cos(z(12, i))+cos(z(12, i))*sin(z(10, i))*sin(z(12, i)))*T * z(3, i+1),...
%         z(9, i+1) == z(9, i) - sin(z(10, i))*T * z(1, i+1) + sin(z(11, i))*cos(z(10, i))*T * z(2, i+1) + cos(z(11, i))*cos(z(10, i))*T * z(3, i+1),...

%         z(7, i+1) == z(7, i) + T * z(1, i+1),...
%         z(8, i+1) == z(8, i) + T * z(2, i+1),...
%         z(9, i+1) == z(9, i) + T * z(3, i+1),...
        
%% ME C231A, EECS C220B, Final Project
clear all
close all
yalmip clear


T = 0.5;
horizon = 60/T;
M = horizon;
x0 = [200,0,0, 0,     0, 5000, 0, 0]';l3
xT = [  50, 0, 0, pi/36, 10000,    0, 0, 0]';
zFinal = xT;

% [feas, xOpt, uOpt, JOpt] = CFTOC_2D_Reference(x0, M, T, zFinal);
[feas, xOpt, uOpt, predErr] = MPC2D(x0, M, xT, T);



plot(xOpt(5,:),xOpt(6,:))
grid on


%% Attribution
% Name: Minglong Li
% Date: 12/03/2017
% Class Number: ME C231A