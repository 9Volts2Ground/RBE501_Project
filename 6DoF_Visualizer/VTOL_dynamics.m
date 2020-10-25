function [sys,x0,str,ts,simStateCompliance] = VTOL_dynamics(t,x,u,flag,P)
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
    sys=mdlDerivatives(t,x,u, P);

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

%
%================================================================
% mdlInitializeSizes
%================================================================
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P)

sizes = simsizes;

sizes.NumContStates  = 6;
sizes.NumDiscStates  = 0; % system parameters
sizes.NumOutputs     = 3;   %h, z, theta
sizes.NumInputs      = 2;   %Fr, Fl
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [P.h0; P.hdot0; P.z0; P.zdot0; P.theta0; P.thetadot0];


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

%
%================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%================================================================
%
function sys=mdlDerivatives(t,x,u,P)
%States
  h = x(1);
  hdot = x(2);
  z = x(3);
  zdot = x(4);
  theta = x(5);
  thetadot = x(6);
  %Inputs
  fr = u(1);
  fl = u(2);
  
  d = P.d;
  Jc = P.Jc;
  mc = P.mc;
  mr = P.mr;
  mu = P.mu;
  g = P.g;
  
  
  hddot = ((fr + fl)*cos(theta) -(mc + 2*mr)*g)/(mc + 2*mr);
    
  zddot = (-(fr + fl)*sin(theta) - mu*zdot)/(mc + 2*mr);

  thetaddot = d*(fr - fl)/(Jc + 2*mr*d^2);

sys = [hdot; hddot; zdot; zddot; thetadot; thetaddot];

% end mdlDerivatives

%
%================================================================
% mdlUpdate
%================================================================
%
function sys=mdlUpdate(t,x,u)
sys = [];

% end mdlUpdate

%
%=================================================================
% mdlOutputs
% Return the block outputs.
%=================================================================
%
function sys=mdlOutputs(t,x,u)
    h = x(1);
    hdot = x(2);
    z = x(3);
    zdot = x(4);
    theta = x(5);
    thetadot = x(6);

    sys = [h; z; theta];

% end mdlOutputs

%
%=================================================================
% mdlGetTimeOfNextVarHit
%=================================================================
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1; % Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
