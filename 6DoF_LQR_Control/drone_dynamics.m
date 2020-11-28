function [sys,x0,str,ts,simStateCompliance] = drone_dynamics(t,x,u,flag,P)
switch flag

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1
    sys=mdlDerivatives(t,x,u,P);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl


%% ========================================================================
% mdlInitializeSizes
%==========================================================================
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P)

sizes = simsizes;

sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0; % system parameters
sizes.NumOutputs     = 12;   %x,y,z,roll,pitch,yaw
sizes.NumInputs      = 4;   %Fr, Fl, Br, Bl
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

% initialize the initial conditions
x0 = [P.position0;
      P.velocity0;
      P.orientation0;
      P.orientation_dot0];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

simStateCompliance = 'UnknownSimState';
% end mdlInitializeSizes


%% ========================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%==========================================================================
function sys=mdlDerivatives(t,x,u,P)
%   Calculates x_dot, the state derivitive
%   Inputs:
%       t - time
%       x - state vector, inertial frame. Position, velocity, orientation, angular velocity
%       u - system inputs (rotor forces)
%       P - struct of simulation parameters
%   Outputs:
%       sys - state dervitive vector

%Unpack States from x vector
X = x(1);
Y = x(2);
Z = x(3);
Xdot = x(4);
Ydot = x(5);
Zdot = x(6);
roll = x(7);
pitch = x(8);
yaw = x(9);
rolldot = x(10);
pitchdot = x(11);
yawdot = x(12);

%Unpack system inputs from u vector
motor_fr = u(1);    %Front right
motor_fl = u(2);    %Front left
motor_br = u(3);    %Back right
motor_bl = u(4);    %Back left

%% Body Torque from Motor Torque
%This is derived experimentally. Need to fill in better system
H1 = motor_fr * P.torque_constant;
H2 = motor_fl * P.torque_constant;
H3 = motor_br * P.torque_constant;
H4 = motor_bl * P.torque_constant;


%% Calculate accelerations
%Note: derivitive of * is *dot, which is given in the input, so we only
%need to calculate the deritivite of given *dot values, which is *ddot

%(Motor force - gravity force)/(total mass)
total_force = motor_fr + motor_fl + motor_br + motor_bl;

%Linear acceleration
Xddot = (cos(pitch)*sin(roll)*sin(yaw) - sin(pitch)*cos(yaw))*total_force/P.mass_total;

Yddot = (cos(pitch)*sin(roll)*cos(yaw) + sin(pitch)*sin(yaw))*total_force/P.mass_total;

Zddot = (cos(pitch)*cos(roll)*total_force - P.mass_total*P.g)/P.mass_total;

%Angular acceleration
rollddot = (P.d*(motor_fr + motor_br - motor_fl  - motor_bl) - (P.Ixx - P.Izz)*pitchdot*yawdot)/P.Iyy;

pitchddot = (P.d*(motor_fr + motor_fl - motor_br - motor_bl) - (P.Izz - P.Iyy)*rolldot*yawdot)/P.Ixx;

yawddot = ( (H1 + H4) - (H2 + H3) - (P.Iyy - P.Ixx)*rolldot*pitchdot )/P.Izz;

% fprintf('t, yawddot, H: %d, %d, %d, %d, %d, %d\n', t, yawddot, H1, H2, H3, H4)



%Pack up state derivitives for output
sys =  [Xdot;
        Ydot;
        Zdot;
        Xddot;
        Yddot;
        Zddot;
        rolldot;
        pitchdot;
        yawdot;
        rollddot;
        pitchddot;
        yawddot];

%Legacy code----------------------------
% %States
%   h = x(1);
%   hdot = x(2);
%   z = x(3);
%   zdot = x(4);
%   theta = x(5);
%   thetadot = x(6);
  
% %Inputs
% fr = u(1);
% fl = u(2);
  
% d = P.d;
% Jc = P.Jc;
% mc = P.mc;
% mr = P.mr;
% mu = P.mu;
% g = P.g;
  
%   hddot = ((fr + fl)*cos(theta) -(mc + 2*mr)*g)/(mc + 2*mr);  
%   zddot = (-(fr + fl)*sin(theta) - mu*zdot)/(mc + 2*mr);
%   thetaddot = d*(fr - fl)/(Jc + 2*mr*d^2);
% sys = [hdot; hddot; zdot; zddot; thetadot; thetaddot];

% end mdlDerivatives

%% ========================================================================
% mdlUpdate
%==========================================================================
function sys=mdlUpdate(t,x,u)
sys = [];

% end mdlUpdate


%% ========================================================================
% mdlOutputs
% Return the block outputs.
%==========================================================================
function sys=mdlOutputs(t,x,u)

%Unpack States from x vector
X = x(1);
Y = x(2);
Z = x(3);
Xdot = x(4);
Ydot = x(5);
Zdot = x(6);
roll = x(7);
pitch = x(8);
yaw = x(9);
rolldot = x(10);
pitchdot = x(11);
yawdot = x(12);

%Reassign states for output
sys =  [X;
        Y;
        Z;
        Xdot;
        Ydot;
        Zdot;
        roll;
        pitch;
        yaw;
        rolldot;
        pitchdot;
        yawdot];

% end mdlOutputs

%% ========================================================================
% mdlGetTimeOfNextVarHit
%==========================================================================
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1; % Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%% ========================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%==========================================================================
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
