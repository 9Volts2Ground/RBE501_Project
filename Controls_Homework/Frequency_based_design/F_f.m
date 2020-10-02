clc; close all; format compact; 


%% Dynamics
F2H_num = [1];
F2H_den = [P.mc + 2*P.mr, 0, 0];
F2H = tf(F2H_num,F2H_den)

%% Lead Controller

zhlead = wnh*.7;
phlead = zhlead*8;

Leadh_num = [1/zhlead 1];
Leadh_den = [1/phlead 1];
Leadh = tf(Leadh_num,Leadh_den)

Hsys = Leadh*F2H

Kh = .128 %.141;

% figure()
% rlocus(Hsys)
% hold on 
% rlocus(Hsys,Kh,'r^')
% grid on
% sgrid([zeta zeta],[wnh 1000])
% xlim([-2,1])
% ylim([-1,1])
% title('H rlocus')