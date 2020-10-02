clc; clear; close; format compact;

run('vtol_param.m')
load('F_Inner_Control.mat')
load('F_h_Control.mat')

%% Z - Lateral Outer Control

Theta2Z_num = [P.g];
Theta2Z_den = [1, P.mu/(P.mc + 2*P.mr), 0];
Theta2Z = tf(Theta2Z_num,Theta2Z_den);

Pout = Theta2Z*(Tau2Th*C_inner/(1 + Tau2Th*C_inner));

figure()
margin(Pout)
grid on
hold on

%% Cutoff Frequency
wc = 1;
wn =  100;

gamma_n = 1e-5;
PC_mag_des_100 = gamma_n


%% PI Control
    %Reject constant input disturbance
ki = .1;

PI_num = [1, ki];
PI_den = [1, 0];
PItf = tf(PI_num, PI_den);

C = PItf;

margin(Pout*C)
legend('Plant','PI')

%% Lead Design
M = 50;

Lead_num = [1, wc/sqrt(M)];
Lead_den = [1, wc*sqrt(M)];
Leadtf = M*tf(Lead_num,Lead_den);

[Lead_mag,~] = bode(Pout*Leadtf,wc);
LeadK = 1/Lead_mag;

C = C*Leadtf*LeadK;

margin(Pout*C)
legend('Plant','PI','Lead')

%% Low Pass Filter

wfilter = 40;
Filter_num = wfilter;
Filter_den = [1, wfilter];
Filtertf = tf(Filter_num,Filter_den);

C = C*Filtertf;

margin(Pout*C);
legend('Plant','PI','Lead','Filter')

%% Constraint Check
PC_mag_wn = bode(Pout*C,wn)
PC_mag_wc = bode(Pout*C,wc)

C_out = C;

[C_out_num,C_out_den] = tfdata(C_out,'v');

%% Constraints
%{
wc = 1 rad/s

Rejects constant input disturbance

wn = 100, gamma_n = 10e-5;

PM = 60

Use a pre-filter
%}