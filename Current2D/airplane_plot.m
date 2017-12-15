function airplane_plot(zOpt_Reference,zOpt_Mpc)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot the motion of a plane according to the output of the CFTOC_2D_Tracking
% required states include x-postion(zOpt_Reference(5,:)), z-position(zOpt_Reference(6,:)) and
% rotation angle(zOpt_Reference(4,:)).
% In the plot, red is the front of the vehicle

% Sampling parameter
rep=2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


t=1;
xsim_reference=[];
xsim_reference(1,1)=zOpt_Reference(5,1);
xsim_reference(2,1)=zOpt_Reference(6,1);
xsim_reference(3,1)=zOpt_Reference(4,1);

xsim_mpc=[];
xsim_mpc(1,1)=zOpt_Mpc(5,1);
xsim_mpc(2,1)=zOpt_Mpc(6,1);
xsim_mpc(3,1)=zOpt_Mpc(4,1);
N=size(zOpt_Reference,2) - 1;

% dividing the interval between two statee into "rep" amount to make it smoother. 
for k = 1:N        
    for j = 1:rep        
         t=t+1;
         % smoother for the reference state
         xsim_reference(1,t) = xsim_reference(1,t-1) + (zOpt_Reference(5,k+1)-zOpt_Reference(5,k))/rep;
         xsim_reference(2,t) = xsim_reference(2,t-1) + (zOpt_Reference(6,k+1)-zOpt_Reference(6,k))/rep;
         xsim_reference(3,t) = xsim_reference(3,t-1) + (zOpt_Reference(4,k+1)-zOpt_Reference(4,k))/rep;
         % smoother for the mpc output state
         xsim_mpc(1,t) = xsim_mpc(1,t-1) + (zOpt_Mpc(5,k+1)-zOpt_Mpc(5,k))/rep;
         xsim_mpc(2,t) = xsim_mpc(2,t-1) + (zOpt_Mpc(6,k+1)-zOpt_Mpc(6,k))/rep;
         xsim_mpc(3,t) = xsim_mpc(3,t-1) + (zOpt_Mpc(4,k+1)-zOpt_Mpc(4,k))/rep;
    end
end

X=xsim_mpc';
Xreference = xsim_reference';

% 1.4) Place Model (for Dynamic Programming)

auto.w = 10.0;                % plane width [m]
auto.l = 70.0;                % plane length [m]
auto.db = 36;               % rear axis position, from back [m]
auto.df = 34;               % front axis position, from front [m]
auto.tyr = 0.8;              % tyre diameter [m]
auto.dmax = 25*pi/180;       % maximum front wheel steering angle [rad]
auto.drat = 14.5;            % ratio between steering wheel angle and front wheel angle
auto.d = auto.l - auto.df - auto.db;  % axis distance [m]

%Compute axis limits here
xmin=min(xsim_reference(1,:));
xmax=max(xsim_reference(1,:));
ymin=min(xsim_reference(2,:));
ymax=max(xsim_reference(2,:));
L=max(xmax-xmin,ymax-ymin);
% limits=[(xmin+xmax)/2-L/2-2,(xmin+xmax)/2+L/2+2,(ymin+ymax)/2-L/2-2,(ymin+ymax)/2+L/2+2];
%[ylowerbound,yupperbound] = ylbd(xsim_reference(2,1),100,200);
%limit = [xsim_reference(1,1) - 300,xsim_reference(1,1) + 300, ylowerbound, yupperbound];
fig = figure();
hold on
% axis(limit)
% xlabel('x [m]')
% ylabel('y [m]')
% title('Path Following')


% Trajectory
plottraj.linestyle = '-';
plottraj.linewidth = 1;
plottraj.color = [0 0 1]; % RGB value
plottraj.marker = 'none';

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Vehicle trajectory (oversampled, to show the path)
for i = 1:N*rep+1
    [p1,p2] = plotcar(X(i,1),X(i,2),X(i,3),0,auto,fig,[0.3 0.3 0.3]);
    pause(0.002)
    if i<=N*rep && i>1 
        delete(p1)
        delete(p2)
    end
    if i<N*rep 
        
        subplot(2,1,1)
        plot(Xreference(:,1),Xreference(:,2),'r');
        plot([X(i,1),X(i+1,1)],[X(i,2),X(i+1,2)],plottraj);
        [ylowerbound,yupperbound] = ylbd(X(i,2),100,200);
        axis([X(i,1)-300,X(i,1)+300,ylowerbound,yupperbound])
        xlabel('x [m]')
        ylabel('y [m]')
        title('Path Following(Zoomed In)')
        hold on
        subplot(2,1,2)
        plot(Xreference(:,1),Xreference(:,2),'r');
        plot([X(i,1),X(i+1,1)],[X(i,2),X(i+1,2)],plottraj);
        axis([0,12500,-5,3500])
        xlabel('x [m]')
        ylabel('y [m]')
        title('Path Following(Global Scale)')
        
        
    end
end
for i = 1:rep:N*rep+1
    subplot(2,1,1)
    plot(X(i,1),X(i,2),'go')
    [ylowerbound,yupperbound] = ylbd(X(i,2),100,200);
    axis([X(i,1)-300,X(i,1)+300,ylowerbound,yupperbound])
    [u,v] = pol2cart(X(i,3),1);
    quiver(X(i,1),X(i,2),u,v,'g','linewidth',2,'maxheadsize',2);
    xlabel('x [m]')
    ylabel('y [m]')
    title('Path Following(Zoomed In)')
    hold on
    subplot(2,1,2)
    plot(X(i,1),X(i,2),'go')
    axis([0,12500,-5,3500])
    xlabel('x [m]')
    ylabel('y [m]')
    title('Path Following(Global Scale)')
    
end

%%%%%%
% Plot model states

% Georg Schildbach, 15/Dec/2013 --- Automobile
% Plots the geometric data of the plane
% --------------------------------------------------------------------------------------------------
% plotcar(x,y,phi,delta,auto,fig,rgb)
% --------------------------------------------------------------------------------------------------
% x,y,phi: coordinates of the plane (in [m], [m], [rad])
% delta: steering angle [rad]
% auto: structure with geometric data of the plane
% fig: figure handle
% rgb: red-green-blue value (1x3 vector)
% --------------------------------------------------------------------------------------------------
% P: plot handle (5x1 vector)
% --------------------------------------------------------------------------------------------------

function [P1,P2] = plotcar(x,y,phi,delta,auto,fig,rgb)

% 1) Compute Car Geometry --------------------------------------------------------------------------

ageom = autogeometry(x,y,phi,delta,auto);
 
% 2) Plot Options ----------------------------------------------------------------------------------

% 2.1) Car
car_op.linestyle = '-';
car_op.color = rgb; % RGB value
car_op.marker = 'none';
car_op.linewidth = 2;


% 2.3) Obstacles
obstacle_op.linestyle = 'none';
obstacle_op.marker = '.';
obstacle_op.markersize = 7;
obstacle_op.markeredgecolor = [0 0 0]; 
obstacle_op.markerfacecolor = [0 0 0];

% 3) Plot ------------------------------------------------------------------------------------------

figure(fig)
hold on
subplot(2,1,1)
P1 = plot([ageom.fr(1),ageom.fl(1),ageom.wing2(1),ageom.wing3(1),ageom.wing1(1),ageom.br(1),ageom.fr(1)],...
          [ageom.fr(2),ageom.fl(2),ageom.wing2(2),ageom.wing3(2),ageom.wing1(2),ageom.br(2),ageom.fr(2)],car_op);
P1 = [P1 ; plot([ageom.sidewing1(1),ageom.sidewing2(1),ageom.sidewing3(1),ageom.sidewing4(1),ageom.sidewing1(1)],...
                [ageom.sidewing1(2),ageom.sidewing2(2),ageom.sidewing3(2),ageom.sidewing4(2),ageom.sidewing1(2)],car_op )];
[ylowerbound,yupperbound] = ylbd(y,100,200);            
axis([x - 300,x + 300, ylowerbound, yupperbound])
xlabel('x [m]')
ylabel('y [m]')
title('Path Following(Zoomed In)')


subplot(2,1,2)

P2 = plot([ageom.fr(1),ageom.fl(1),ageom.wing2(1),ageom.wing3(1),ageom.wing1(1),ageom.br(1),ageom.fr(1)],...
          [ageom.fr(2),ageom.fl(2),ageom.wing2(2),ageom.wing3(2),ageom.wing1(2),ageom.br(2),ageom.fr(2)],car_op);
P2 = [P2 ; plot([ageom.sidewing1(1),ageom.sidewing2(1),ageom.sidewing3(1),ageom.sidewing4(1),ageom.sidewing1(1)],...
                [ageom.sidewing1(2),ageom.sidewing2(2),ageom.sidewing3(2),ageom.sidewing4(2),ageom.sidewing1(2)],car_op )]; 
axis([0,12500,-5,3500])
xlabel('x [m]')
ylabel('y [m]')
title('Path Following(Global Scale)')

end

% ==================================================================================================

% Georg Schildbach, 12/Dec/2013 --- Automobile
% Computes the plane circumfence, based on current coordinates and geometric data
% --------------------------------------------------------------------------------------------------
% ageom = autogeometry(x,y,phi,delta,auto)
% --------------------------------------------------------------------------------------------------
% x,y,phi: coordinates of the plane (in [m], [m], [rad])
% delta: steering angle [rad]
% auto: structure with geometric data of the plane
% --------------------------------------------------------------------------------------------------
% ageom: structure with plane geometry data
% --------------------------------------------------------------------------------------------------

function ageom = autogeometry(x,y,phi,delta,auto)

% 1) Linear Constraints ----------------------------------------------------------------------------

% 1.1) Vectors

ageom.gf = [+cos(phi) ; +sin(phi)];
ageom.gb = [-cos(phi) ; -sin(phi)];
ageom.gr = [+sin(phi) ; -cos(phi)];
ageom.gl = [-sin(phi) ; +cos(phi)];

% 1.2) Offsets

ageom.hf =  x*cos(phi) + y*sin(phi) + auto.d + auto.df;
ageom.hb = -x*cos(phi) - y*sin(phi) + auto.db;
ageom.hr =  x*sin(phi) - y*cos(phi) + auto.w/2;
ageom.hl = -x*sin(phi) + y*cos(phi) + auto.w/2;



% 2) Corners ---------------------------------------------------------------------------------------

ageom.fr = [x + (auto.d+auto.df)*cos(phi) ; y + (auto.d+auto.df)*sin(phi)];
ageom.fl = [x + (auto.d+auto.df)*cos(phi) ; y + (auto.d+auto.df)*sin(phi)] + auto.w*ageom.gl;
ageom.br = [x - 33/2*cos(phi) ; y - 33/2*sin(phi)] ;
%ageom.br = [x - auto.db*cos(phi) ; y - auto.db*sin(phi)] + auto.w*ageom.gr ;
%ageom.bl = [x - auto.db*cos(phi) ; y - auto.db*sin(phi)] + auto.w/2*ageom.gl;

% Wings --------------------------------
ageom.wing1 = [x - auto.db*cos(phi) ; y - auto.db*sin(phi)] + auto.w*ageom.gl;
ageom.wing2 = ageom.wing1 + 20*ageom.gf;
ageom.wing3 = ageom.wing1 + 10*ageom.gl;

% WWWWWWWings
ageom.sidewing1 = [x;y] + 33/2 * ageom.gf;
ageom.sidewing2 = [x;y] + 33/2 * ageom.gf + 5 * [sin(phi);cos(phi)];
ageom.sidewing3 = [x;y] - 33/2 * ageom.gf + 5 * [sin(phi);cos(phi)];
ageom.sidewing4 = [x;y] - 33/2 * ageom.gf;


% 3) Tyres -----------------------------------------------------------------------------------------

ageom.sta = [+cos(phi+delta) ; +sin(phi+delta)];
ageom.tfr =  ([x;y]+auto.d*ageom.gf+0.75*auto.w/2*ageom.gr)*[1,1] + ...
                                                   [0.5*auto.tyr*ageom.sta,-0.5*auto.tyr*ageom.sta];
ageom.tfl =  ([x;y]+auto.d*ageom.gf+0.75*auto.w/2*ageom.gl)*[1,1] + ...
                                                   [0.5*auto.tyr*ageom.sta,-0.5*auto.tyr*ageom.sta];
ageom.tbr =  ([x;y]+0.75*auto.w/2*ageom.gr)*[1,1] + [0.5*auto.tyr*ageom.gb,-0.5*auto.tyr*ageom.gb];
ageom.tbl =  ([x;y]+0.75*auto.w/2*ageom.gl)*[1,1] + [0.5*auto.tyr*ageom.gb,-0.5*auto.tyr*ageom.gb];

end

function [ylowerbound,yupperbound] = ylbd(y,downlimit,range)
    if y - downlimit > 0
        ylowerbound = y - downlimit;
    else
        ylowerbound = 0;
    end
    yupperbound = ylowerbound + range;
end

end


%% Attribution
% ME C231A and EECS C220B, UC Berkeley, Fall 2016