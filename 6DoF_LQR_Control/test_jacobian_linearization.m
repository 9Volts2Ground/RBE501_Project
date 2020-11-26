clc; clear all; close all; format compact;

drone_param

%% Define symbolic state vector

syms x y z xdot ydot zdot phi theta psi phidot thetadot psidot real
X = [x y z xdot ydot zdot phi theta psi phidot thetadot psidot]';

syms u1 u2 u3 u4 real
u = [u1 u2 u3 u4]';

%% Define acceleration equations
%Linear acceleration
xddot = (cos(theta)*sin(phi)*sin(psi) - sin(theta)*cos(psi))*sum(u)/P.mass_total;

yddot = (cos(theta)*sin(phi)*cos(psi) + sin(theta)*sin(psi))*sum(u)/P.mass_total;

zddot = (cos(theta)*cos(phi)*sum(u) - P.mass_total*P.g)/P.mass_total;

%Angular acceleration
phiddot = (P.d*(u1 + u3 - u2  - u4) - (P.Ixx - P.Izz)*thetadot*psidot)/P.Iyy;

thetaddot = (P.d*(u1 + u2 - u3 - u4) - (P.Izz - P.Iyy)*phidot*psidot)/P.Ixx;

psiddot = ( (u1* P.torque_constant + u4* P.torque_constant) - (u2* P.torque_constant + u3* P.torque_constant) - (P.Iyy - P.Ixx)*phidot*thetadot )/P.Izz;

state_dir = [xdot, ydot, zdot, xddot, yddot, zddot, phidot, thetadot, psidot, phiddot, thetaddot, psiddot]';

%% Solve Jacobian Linearization
A = sym( zeros( length(state_dir), length(X) ) );
for der_ind = 1:1:length(state_dir)
    for state_ind = 1:1:length(X)
        A(der_ind, state_ind) = simplify( diff( state_dir(der_ind), X(state_ind) ) );
    end
end

B = sym( zeros( length(state_dir), length(u) ) );
for der_ind = 1:1:length(state_dir)
    for u_ind = 1:1:length(u)
        B(der_ind, u_ind) = simplify( diff( state_dir(der_ind), u(u_ind) ) );
    end
end

%% Define equilibrium values

x_equilibrium = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]';
u_equilibrium = [P.mass_total*P.g/4, P.mass_total*P.g/4, P.mass_total*P.g/4, P.mass_total*P.g/4]';

%% Substitute equilibrium points into state matricies
A_equilib = sym( zeros( size( A ) ) );
for der_ind = 1:1:length(state_dir)
    for state_ind = 1:1:length(X)
        A_equilib(der_ind, state_ind) = simplify( subs( A(der_ind, state_ind), [X; u], [x_equilibrium; u_equilibrium] ) );
    end
end

B_equilib = sym( zeros( size( B ) ) );
for der_ind = 1:1:length(state_dir)
    for u_ind = 1:1:length(u)
        B_equilib(der_ind, u_ind) = simplify( subs( B(der_ind, u_ind), [X; u], [x_equilibrium; u_equilibrium] ) );
    end
end




