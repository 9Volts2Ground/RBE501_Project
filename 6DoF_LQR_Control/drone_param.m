clc; clear all; close all; format compact;
%% ========================================================================
%Simulation Paramters
%==========================================================================

%% Desired state
P.X_des = [1, 2, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0]';

%% Plotting parameters
% Size of the Done
P.w = 0.3048;           %width of the base body in meters (12 inches)
P.base_height = 0.0508; %Thickness of the body in meters (about 2")

P.prop_length = 0.254; %Diameter of propellers, in meters (10 inches)
P.prop_thickness = 0.00635; %Thickness of propeller plot, in meters (1/4 inch)

%% Drone physical properties
%Distance from center to motor center (assume motor center = motor center of mass)
P.d = P.w*sqrt(2)/2;

%Masss properties (need to be updated with realistic hardware values)
P.body_mass = 1;    %Drone frame mass
P.Jc = .0042;       %Drone frame inertia
P.motor_mass = .5;  %Motor mass
P.prop_mass = .1;   %Prop mass
P.mass_total = P.body_mass + 4*(P.motor_mass + P.prop_mass); 

%Inertia values (arbitrarily chosen)
P.Ixx = 1;
P.Iyy = 1;
P.Izz = 1;

%% Initial Conditions
P.position0 = [0, 0, 0]';
P.velocity0 = [0, 0, 0]';
P.orientation0 = [0, 0, 0]';
P.orientation_dot0 = [0, 0, 0]';

%% Universe constants
P.g = 9.81;     %Gravity

%% Equilibrium
P.force_equilibrium = P.mass_total * P.g;
P.motor_equilibrium = P.force_equilibrium/4;
P.roll_equilibrium = 0;
P.pitch_equilibrium = 0;
P.yaw_equilibrium = 0;

P.gravity_equilibrium = [P.motor_equilibrium, P.motor_equilibrium, P.motor_equilibrium, P.motor_equilibrium]';

%% Max actuator limits
P.Force_max = 20;
P.Torque_max = 10;

%% Body Torque from Motor Torque
%This is derived experimentally. Value is assumed for now
P.torque_constant = 50;

%% LQR Control

%Linearized State-Space Model
P = drone_linearization(P);

%Cost matrices. Default to values of 1 (identity matrices)
    %Q = state cost
    %R = actuator cost
P.Q = eye(12);

P.R = eye(4);

%Updated gain values
P.Q(1,1) = 4;
P.Q(2,2) = 4;
P.Q(3,3) = 4;

%% Grab LQR gain matrix
sprintf('Calculating LQR gain matrix K... \n')
P.K = lqr(P.A, P.B, P.Q, P.R);

%% Noise parameters
P.feedback_noise = false;
P.noise_sigma = 0.25;

%% Input Disturbance
P.input_disturbance = true;


%% Simulation Parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 15.0;   % End time of simulation
P.Ts = 0.01;      % sample time for simulation
P.t_plot = 0.02;   % the plotting and animation is updated at this rate

fprintf('Done initializing. Running sim...')

%% Legacy code------------------------------------------------------------

% %Size of the Target
% P.wt = .5;

% %Equilibrium force/torque
% P.fr_equal = (P.mc + P.mr + P.ml)*P.g/2;
% P.Fe = P.g*(P.mc + P.mr + P.ml);
% P.Tau_equal = 0;

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

% %Feedback Gains
% trh = 8;
% zetah = .707;
% wnh = 2.2/trh;
% 
% trth = .8;
% zetath = .707;
% wnth = 2.2/trth;
% 
% trz = 10*trth;
% zetaz = .707;
% wnz = 2.2/trz;

% P.kptheta = wnth^2*(P.Jc + 2*P.mr*P.d^2);
% P.kdtheta = 2*zeta*wnth*(P.Jc + 2*P.mr*P.d^2);
% 
% trz = 10*trth;
% wnz = 2.2/trz;
% P.kpz = -wnz^2/P.g;
% P.kdz = (P.mu/(P.mc + 2*P.mr) - 2*zeta*wnz)/P.g;

% b = P.mc + 2*P.mr;
% % State Models
%     %x_h = [h hdot]';
% P.Ah = [0 1;
%         0 0];
% P.Bh = [0, 1/b]';
% P.Ch = [1 0];
% P.Dh = 0;
% 
%     %x_z = [z theta zdot thetadot]';
% P.Az = [0 0 1 0;
%         0 0 0 1;
%         0, -P.Fe/b, -P.mu/b, 0;
%         0 0 0 0];
% P.Bz = [0 0 0 1/(P.Jc + 2*P.mr*P.d^2)]';
% P.Cz = [1 0 0 0;
%         0 1 0 0];
% P.Dz = [0];
%       
% P.Ch_AB = [P.Bh, P.Ah*P.Bh];
% P.Cz_AB = [P.Bz, P.Az*P.Bz, P.Az*P.Az*P.Bz, P.Az*P.Az*P.Az*P.Bz];

% %Dirty Derivative
% P.sigma = .05;
% P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts);

