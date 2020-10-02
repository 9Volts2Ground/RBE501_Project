clc; close all; format compact;

%% F.11.a - Find Poles

tf_num = [1];
h_den = [1, 2*zetah*wnh, wnh^2];
th_den = [1, 2*zetath*wnth, wnth^2];
z_den = [1,2*zetaz*wnz, wnz^2];

h_tf = tf(tf_num,h_den);
th_tf = tf(tf_num,th_den);
z_tf = tf(tf_num,z_den);

F_sys = h_tf
Tau_sys = th_tf*z_tf

Poles_h = pole(F_sys)
Poles_z = pole(Tau_sys)

%% F.11.c - Verify rank

n_h = rank(P.Ch_AB)
n_z = rank(P.Cz_AB)

%% F.11.d - Find K

%% h
Kh = place(P.Ah,P.Bh,Poles_h)
Eig_h = eig(P.Ah - P.Bh*Kh)
Krh = -1/(P.Ch*inv(P.Ah - P.Bh*Kh)*P.Bh)
% Store values to run in Simulink
P.Krh = Krh;
P.Kh = Kh;

%% z
Kz = place(P.Az,P.Bz,Poles_z)
Eig_z = eig(P.Az - P.Bz*Kz)
Krz = -1/(P.Cz(1,:)*inv(P.Az - P.Bz*Kz)*P.Bz)

% Store values to run in Simulink
P.Krz = Krz;
P.Kz = Kz;