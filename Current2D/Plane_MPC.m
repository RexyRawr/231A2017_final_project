function [xOpt, uOpt, feas] = Plane_MPC(step, horizon, T, x0, xT, xOpt_R)
g = 9.81;

% constraint relaxation factor
delta_xT = [5,2.5,pi/180,pi/90,20,5,g/10,g/20]';

feas = false(1,horizon);
% eight states: z = (u,w,Q,theta,X,Z, Ax, Az)
xOpt = zeros(8,horizon+1);
% three inputs: u = (X,Y,M)
uOpt = zeros(3,horizon);
xOpt(:,1) = x0;

% Gaussian noise
noise = makedist('normal', 'mu', 0, 'sigma', sqrt(3)); 

% Run MPC
for i = 1 : step : 120
    
    xOpt_R(2,i) = xOpt_R(2,i)+xOpt_R(2,i)*random(noise)*0.05;

    [feas_Temp, xOpt_Temp, uOpt_Temp, ~] = CFTOC_2D_Tracking(xOpt(:,i), horizon + 1 - i, T, xT, xOpt_R(:,i:end),delta_xT,0);

    feas(i) = feas_Temp;
    if feas_Temp
        xOpt(:,i+1:i+step) = xOpt_Temp(:,2:step + 1);
        uOpt(:,i:i+step-1) = uOpt_Temp(:,1:step);   
    else
        xOpt = [];
        uOpt = [];
        return
    end
    
end

end