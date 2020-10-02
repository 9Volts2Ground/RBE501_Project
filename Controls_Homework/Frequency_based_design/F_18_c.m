clc; clear; close; format compact;

run('vtol_param.m')
load('F_Inner_Control.mat')

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
PC_mag_des = gamma_n

%% Lead Design
M = 20;

Lead_num = [1, wc/sqrt(M)];
Lead_den = [1, wc*sqrt(M)];
Leadtf = M*tf(Lead_num,Lead_den);

[Lead_mag,~] = bode(Pout*Leadtf,wc);
LeadK = 1/Lead_mag;

margin(Pout*Leadtf*LeadK)
legend('Plant','Lead')

%% Low Pass Filter

wfilter = 15;
Filter_num = wfilter;
Filter_den = [1, wfilter];
Filtertf = tf(Filter_num,Filter_den);

margin(Pout*Leadtf*LeadK*Filtertf);
legend('Plant','Lead','Filter')

%% Constraint Check
PC_mag_wn = bode(Pout*Leadtf*LeadK*Filtertf,wn)

C_out = Leadtf*LeadK*Filtertf

[C_out_num,C_out_den] = tfdata(C_out)

%% Constraints
%{
wc = 1 rad/s

Rejects constant input disturbance

wn = 100, gamma_n = 10e-5;

PM = 60

Use a pre-filter
%}