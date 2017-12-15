% this file gives the dynamic model of a 2D aircraft. The input will be the
% force in x-direction(X) and z-direction(Z), also, there will be pitch
% moment(M). Six states are required, the velocity in the x-direction(u),
% the velocity in the z-direction(w), pitch rate(Q), global x position(x),
% global z position(z) and pitch angle(theta). Time step will be noted as
% T.
%
% m(udot + Q*w + g*sin(theta)) = X
% m(wdot - Q*u - g*cos(theta)) = Z
% Iy*Qdot = M
%
% u(k+1) = u(k) + T * (X/m - Q*w - g*sin(theta))
% w(k+1) = w(k) + T * (Z/m + Q*u + g*cos(theta))
% Q(k+1) = Q(k) + T * M/Iy
%
% theta(k+1) = theta(k) + T * Q(k)
% x(k+1) = x(k) + T*(u(k)*cos(theta(k)) + w(k)*sin(theta(k))
% z(k+1) = z(k) + T*(u(k)*sin(theta(k)) + w(k)*cos(theta(k))
% After checking the following code, try to run with the below code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [feas_R, xOpt_R, uOpt_R, JOpt_R] = CFTOC_2D_Reference(x0, step1, T, zFinal)
g = 9.81;
m = 288773.23;
Iy = 44877574.15;

% Define state variables
% z = (u,w,Q,theta,X,Z, Ax, Az)
sizex = 8;
z = sdpvar(sizex,step1+1);
assign(z(:,1),x0);

% Define decision variables
% u = (X,Z,M)
sizeu = 3;
u = sdpvar(sizeu,step1);

% initializing objective and constraints
objective = 0;
constraints = [z(:, 1)==x0];

for i = 1:step1
    
    % penalizing work used, which will be power * T = Force * velocity * T and Torque * angular velocity * T 
    objective = objective + abs(T*u(1,i)*z(1,i)) + abs(T * u(2, i) * z(2, i)) + abs(T * u(3, i) * z(3, i));
    
    constraints = [constraints,... 
                   % u(k+1) = u(k) + T * (X/m - Q*w - g*sin(theta))
                   z(1,i+1) == z(1,i) + T * (u(1,i)/m - z(3,i)*z(2,i) - g*sin(z(4,i))),...
                   % w(k+1) = w(k) + T * (Z/m + Q*u + g*cos(theta))
                   z(2,i+1) == z(2,i) + T * (u(2,i)/m - z(3,i)*z(1,i) + g*cos(z(4,i))),...
                   % Q(k+1) = Q(k) + T * M/Iy
                   z(3,i+1) == z(3,i) + T * u(3,i)/Iy,...
                   % theta(k+1) = theta(k) + T * Q(k)
                   z(4,i+1) == z(4,i) + T * z(3,i),...
                   % x(k+1) = x(k) + T*(u(k)*cos(theta(k)) + w(k)*sin(theta(k))
                   z(5,i+1) == z(5,i) + T * (z(1,i)*cos(z(4,i)) + z(2,i)*sin(z(4,i))),...
                   % z(k+1) = z(k) + T*(u(k)*sin(theta(k)) + w(k)*cos(theta(k))
                   z(6,i+1) == z(6,i) + T * (z(1,i)*sin(z(4,i)) + z(2,i)*cos(z(4,i))),...
                   % acceleration Ax(k+1) = (z(1,k+1) - z(1,k))/T 
                   z(7,i+1) == (z(1,i+1) - z(1,i))/T ,...
                   % acceleration Az(k+1) = (z(2,k+1) - z(2,k))/T
                   z(8,i+1) == (z(2,i+1) - z(2,i))/T ,...
                   
                   % landing with touchdown speed 44.24-79.7m/s (1957.17-6352.09(m/s)^2
                   z(1,i)^2 + z(2,i)^2 >= 1957.17...
                   % z position should always be greater than 0
                   z(6,i) >= 0,...
                   
                   % maximum speed 231.5m/s = 53592.25(m/s)^2
                   z(1,i)^2 + z(2,i)^2 <= 53592.25,...
                   % pitch angle between -10 to +25 degree
                   25 * pi / 180 >= z(4, i) >= -10 * pi / 180,...
                   
                   % acceleration in x direction within +-g
                   g >= z(7,i+1) >= -g,...
                   % acceleration in x direction within +-g/2
                   g/2 >= z(8,i+1) >= -g/2,...
                   % pitch rate with in 5 degree
                   pi/36 >= z(3,i+1) >= -pi/36,...
                   % landing with touchdown speed 44.24-79.7m/s (1957.17-6352.09(m/s)^2
                   z(1,i)^2 + z(2,i)^2 >= 1957.17...
                   % z position should always be greater than 0
                   z(6,i) >= 0];
                   
                   
end
    constraints = [constraints, z(1,step1+1) == zFinal(1),...
                                z(2,step1+1) == zFinal(2),...
                                z(4,step1+1) == zFinal(4),...
                                z(5,step1+1) == zFinal(5),...
                                z(6,step1+1) == zFinal(6),...
                                z(8,step1+1) == zFinal(8)];


% Set options for YALMIP and solver
options = sdpsettings('verbose',0,'solver','ipopt');

% Solve
sol = optimize(constraints, objective, options);

% Retrieve solutions
opt_z = value(z);
opt_u = value(u);
opt_J = value(objective);

xOpt_R = opt_z;
uOpt_R = opt_u;
JOpt_R = opt_J;
feas_R = true;

if sol.problem == 1
    feas_R = false;
    xOpt_R = [];
    uOpt_R = [];
    JOpt_R = [];
end
end