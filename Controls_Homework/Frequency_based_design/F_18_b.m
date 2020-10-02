clc; clear; close; format compact;

run('vtol_param.m')

%% Tau2Theta - Lateral Inner Control

Theta2Z_num = [P.g];
Theta2Z_den = [1, P.mu/(P.mc + 2*P.mr), 0];
Theta2Z = tf(Theta2Z_num,Theta2Z_den);

figure()
bode(Theta2Z)
grid on
hold on

%% Cutoff Frequency 
wc = 11;

%% wc Gain
% PC_mag_wc = bode(Theta2Z,wc);
% K = 1/PC_mag_wc;
% 
% margin(Theta2Z*K)
% legend('Plant','Offset Gain')

%% Lead Design
M = 500;

Lead_num = [1, wc/sqrt(M)];
Lead_den = [1, wc*sqrt(M)];
Leadtf = M*tf(Lead_num,Lead_den);

[Lead_mag,~] = bode(Theta2Z*Leadtf,wc);
LeadK = 1/Lead_mag;
% PC_mag_wr = bode(Phtf*Leadtf,wr);
% LeadK = PC_mag_des/PC_mag_wr;

margin(Theta2Z*Leadtf*LeadK)
legend('Plant','Lead')

%% Low Pass Filter

wfilter = 21;
Filter_num = wfilter;
Filter_den = [1, wfilter];
Filtertf = tf(Filter_num,Filter_den);

margin(Theta2Z*Leadtf*LeadK*Filtertf);
legend('Plant','Lead','Filter')


%% Save Controller
C_inner = Leadtf*LeadK*Filtertf

save('F_Inner_Control.mat','C_inner','Theta2Z')

%% Constraints
%{
Stable

PM = 60 degrees

wc = 10 rad/s

Low pass filter
%}