function [A, B, C, x0] = load_parameter(a, b, c, d)
%   parameter a, b, c, d
% a = 3;
% b = 1;
% c = 1;
% d = 5;

%   alpha, beta, gamma, delta
alpha = 15.5 - a / 3 + b / 2;
beta = 27.5 - d / 2;
gamma = 11.5 + (a - c) / (b + d + 3);
delta = 60 + (a - b) * c / 10;

%   g
g = 9.8;

%   Mass of each part
M_f = 2.14 + c / 20;
M_r = 5.91 - b / 10;
M_c = 1.74;

%   Vertical length from a floor to a center-of-gravity of each part
H_f = 0.18;
H_r = 0.161;
H_c = 0.098;

%   Horizontal length from a front wheel rotation axis to a center-of-gravity of 
%   part of front wheel and steering axis.
L_Ff = 0.05;
L_F = 0.133;

%   Horizontal length from a rear wheel rotation axis to a center-of-gravity of 
%   part of rear wheel and steering axis.
L_r = 0.128;
L_R = 0.308 + (a - d) / 100;

%   Horizontal length from a rear wheel rotation axis to a center-of-gravity of the cart system
L_c = 0.259;

%   Moment of inertia around center-of-gravity x axially
J_x = 0.5 + (c - d) / 100;

%   Moment of inertia for part of front wheel z axially.
% J_fz = 

%   Moment of inertia for part of rear wheel that contains cart system z axially.
% J_z

%   Viscous coefficient around x axis.
miu_x = 3.33 - b / 20 + a * c / 60;

%   Viscous coefficient for part of front wheel around z axis.
% miu_fz

%   Viscous coefficient for part of rear wheel that contains cart system around z axis.
% miu_z

%   A viscosity coefficient of a movement direction of the cart system
% miu_c

%   Part of front wheel, rear wheel, and cart system respectively
% f
% r
% c

%   Cart position, handle angle and bike angle
% d_t
% phi_t
% psai_t

%   parameter in matrix
den = M_f * H_f * H_f + M_r * H_r * H_r + M_c * H_c * H_c + J_x;
a51 = - M_c * g / den;
a52 = (M_f * H_f + M_r * H_r + M_c * H_c) * g / den;
a53 = (M_r * L_r * L_F + M_c * L_c * L_F + M_f * L_Ff * L_R) * g / (L_R + L_F) / den;
a54 = - M_c * H_c * alpha / den;
a55 = - miu_x / den;
a56 = M_f * H_f * L_Ff * gamma / den;
b51 = M_c * H_c * beta / den;
b52 = - M_f * H_f * L_Ff * delta / den;

A = [0      0      0      1      0      0;
     0      0      0      0      1      0;
     0      0      0      0      0      1;
     0    6.5    -10   -alpha    0      0;
    a51   a52    a53    a54    a55    a56;
     5   -3.6      0      0      0   -gamma];

B = [0      0;
     0      0;
     0      0;
     beta  11.2;
     b51   b52;
     40    delta];

C = [1   0   0   0   0   0;
     0   1   0   0   0   0;
     0   0   1   0   0   0];

x0 = [0.2;  -0.1;   0.15;  -1;  0.8;   0];

end



