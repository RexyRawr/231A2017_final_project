function [feas_T, xOpt_T, uOpt_T, JOpt_T] = CFTOC_2D_Tracking(x0, step, T, zFinal, reference, delta_xT, relax)
% [feas_T, xOpt_T, uOpt_T, JOpt_T] = CFTOC_2D_Tracking(x0, step, T, zFinal, reference, delta_xT, relax)
% x0 is the initial point
% step is the step per calculation
% T is the time step, in this computation, 0.5s is used
% zFinal is the terminal point
% reference is the path wished to follow
% delta_xT is the relaxation factor once typical terminal constraint is infeasible
% relax is used to determine whether or not relaxation have already been operated

g = 9.81;
m = 288773.23;
Iy = 44877574.15;

% Define state variables
% z = (u,w,Q,theta,X,Z, Ax, Az)
sizex = 8;
z = sdpvar(sizex,step+1);
assign(z(:,1),x0);

% Define decision variables
% u = (X,Z,M)
sizeu = 3;
u = sdpvar(sizeu,step);

% initializing objective and constraints
objective = 0;
constraints = [z(:, 1)==x0];

for i = 1:step
    
    % During Tracking, penalize the postion. x, z and the velocities in
    % both directions
    objective = objective +2*(z(5,i) - reference(5,i))^2 + 2*(z(6,i) - reference(6,i))^2+(z(1,i) - reference(1,i))^2+(z(2,i) - reference(2,i))^2;
    
    
    
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
    if relax ==0
        constraints = [ constraints, (zFinal - delta_xT <= z(:,step+1) <= zFinal + delta_xT)];
    else
        fprintf('going into relax constraints\n')
        delta_double = delta_xT.*2;
        constraints = [ constraints, (zFinal -delta_double <= z(:,step+1) <= zFinal + delta_double)];
    end

% Set options for YALMIP and solver
options = sdpsettings('verbose',0,'solver','ipopt');

% Solve
sol = optimize(constraints, objective, options);

% Retrieve solutions
opt_z = value(z);
opt_u = value(u);
opt_J = value(objective);

xOpt_T = opt_z;
uOpt_T = opt_u;
JOpt_T = opt_J;
feas_T = true;

if sol.problem == 1
    if relax == 0
        [feas_T, xOpt_T, uOpt_T, JOpt_T] = CFTOC_2D_Tracking(x0, step, T, zFinal, reference,delta_xT,1);
    else
        xOpt_T = [];
        uOpt_T = [];
        JOpt_T = [];
        feas_T = 0;
    end
end
end