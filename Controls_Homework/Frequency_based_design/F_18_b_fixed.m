clc; clear; close; format compact;

run('vtol_param.m')

%% Tau2Theta - Lateral Inner Control

Tau2Th_num = [1];
Tau2Th_den = [(P.Jc + 2*P.mr*P.d^2), 0, 0];
Tau2Th = tf(Tau2Th_num,Tau2Th_den);

figure()
bode(Tau2Th)
grid on
hold on

%% Cutoff Frequency 
wc = 11;


%% Lead Design
M = 500;

Lead_num = [1, wc/sqrt(M)];
Lead_den = [1, wc*sqrt(M)];
Leadtf = M*tf(Lead_num,Lead_den);

[Lead_mag,~] = bode(Tau2Th*Leadtf,wc);
LeadK = 1/Lead_mag;

margin(Tau2Th*Leadtf*LeadK)
legend('Plant','Lead')

%% Low Pass Filter

wfilter = 21;
Filter_num = wfilter;
Filter_den = [1, wfilter];
Filtertf = tf(Filter_num,Filter_den);

margin(Tau2Th*Leadtf*LeadK*Filtertf);
legend('Plant','Lead','Filter')


%% Save Controller
C_inner = Leadtf*LeadK*Filtertf

[C_inner_num,C_inner_den] = tfdata(C_inner,'v');

save('F_Inner_Control.mat','C_inner','Tau2Th','C_inner_num','C_inner_den')

%% Constraints
%{
Stable

PM = 60 degrees

wc = 10 rad/s

Low pass filter
%}