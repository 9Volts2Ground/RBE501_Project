clc; clear all; close all; format compact;

syms mc mr kd kp

s1 = -.2;
s2 = -.3;

Eq(1) = (0 == (mc + 2*mr)*s1^2 + kd*s1 + kp);
Eq(2) = (0 == (mc + 2*mr)*s2^2 + kd*s2 + kp);

[Kp,Kd] = solve(Eq,kp,kd)