clc; clear all; close all; format compact;
%% Simulation Paramters

% Size of the VTOL
P.w = .2;   %width of the base body
P.ew = .2;  %Ellipse width
P.eh = .05; %Ellipse height

%Size of the Target
P.wt = .5;

%Center of mass and inertia
P.mc = 1;
P.Jc = .0042;

%Right and left masses
P.mr = .25;
P.ml = .25;

%Distance from center to prop masses
P.d = .3;

%Drag damping coefficient and gravity
P.mu = .1;
P.g = 9.81;

% Initial Conditions
P.z0 = 0;
P.zdot0 = 0;
P.h0 = 0;
P.hdot0 = 0;
P.theta0 = 0;
P.thetadot0 = 0;

% %Feedback Gains
% P.kp = 3*(P.mc + P.ml + P.mr)/50;
% P.kd = .5*(P.mc + P.mr + P.ml);
% 
% tr = 8;
% zeta = .7;
% wn = 2.2/tr;

% % %Feedback Gains, F.8a
% P.kp8 = wn^2*(P.mc + 2*P.mr);
% P.kd8 = 2*dr*wn*(P.mc + 2*P.mr);
% 
% tr = .22;
% dr = .7;
% wn = 2.2/tr;

%Feedback Gains
trh = 8;
wnh = 2.2/trh;

trth = .8;
zeta = .7;
wnth = 2.2/trth;

P.kptheta = wnth^2*(P.Jc + 2*P.mr*P.d^2);
P.kdtheta = 2*zeta*wnth*(P.Jc + 2*P.mr*P.d^2);

trz = 10*trth;
wnz = 2.2/trz;
P.kpz = -wnz^2/P.g;
P.kdz = (P.mu/(P.mc + 2*P.mr) - 2*zeta*wnz)/P.g;

%Equilibrium
P.fr_equal = (P.mc + P.mr + P.ml)*P.g/2;
P.F_equal = P.g*(P.mc + P.mr + P.ml);
P.Tau_equal = 0;

% Sets max limits
P.Force_max = 20;
P.Torque_max = 10;

% Simulation Parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 50.0;   % End time of simulation
P.Ts = 0.01;      % sample time for simulation
P.t_plot = 0.005;   % the plotting and animation is updated at this rate

