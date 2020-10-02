clc; close all;

z = wn*.8;
p = 8.8*z;%.8, 8.8

C_num = [1/z 1];
C_den = [1/p 1];
C_sys = tf(C_num,C_den)

G_num = [1];
G_den = [(P.mc + 2*P.mr), 0, 0];
G_sys = tf(G_num,G_den)

sys = C_sys*G_sys

figure()
rlocus(sys)
hold on 
grid on
sgrid([dr dr],[wn 1000])
axis equal

K = .215; %.279;