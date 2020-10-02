clc; clear; close; format compact;

run('vtol_param.m')

%% H - Vertical Plant
Ph_num = 1;
Ph_den = [(P.mc + 2*P.mr), 0, 0];
Phtf = tf(Ph_num, Ph_den);

figure()
bode(Phtf)
grid on
hold on

%% Cutoff Frequency
wr = .1;
wn = 200;
wc = 6.5;

gamma_r = .01;
PC_mag_des = 1/gamma_r

%% PI Control
    %Reject constant input disturbance
ki = .5;

PI_num = [1, ki];
PI_den = [1, 0];
PItf = tf(PI_num, PI_den);

C = PItf;

margin(Ptf*C)
legend('Plant','PI')


%% Lead Design
M = 1000;

Lead_num = [1, wc/sqrt(M)];
Lead_den = [1, wc*sqrt(M)];
Leadtf = M*tf(Lead_num,Lead_den);

PC_mag_wr = bode(Phtf*Leadtf,wr);
LeadK = PC_mag_des/PC_mag_wr;

C = C*Leadtf*LeadK;

margin(Phtf*C)
legend('Plant','PI','Lead')



%% Low Pass Filter

wfilter = 14;
Filter_num = wfilter;
Filter_den = [1, wfilter];
Filtertf = tf(Filter_num,Filter_den);

C = C*Filtertf;

margin(Phtf*C);
legend('Plant','PI','Lead','Filter')

%% Second Low Pass Filter
wfilter2 = 18;
Filter_num2 = wfilter2;
Filter_den2 = [1,wfilter2];
Filtertf2 = tf(Filter_num2,Filter_den2);

C = C*Filtertf2;

margin(Phtf*C)
legend('Plant','PI','Lead','Filter1','Filter2')

%% Constraint Check
PC_mag_wr = bode(Phtf*C,wr) 
PC_mag_wn = bode(Phtf*C,wn) 

C_h = C

%% Constraints
%{
Reject input disturbance - slope at low frequencies ~= 0

Tracking, wr = .1 so that gamma_r = .01

Noise, wn = 200 rad/s, gamma_n = .0001;

PM = 60

Prefilter
%}