%   zita >= ln10 / sqrt(pi*pi + ln10 * ln10) = 0.591
%   zita = 0.707
%   w_n = 1.13
%   H_d(s) = (1.2769) / (s^2 + 1.59782*s + 1.2769)
%   lamda12 = -zita*w_n +- j w_n*sqrt(1-zita*zita);
function [K11,K] = pole_placement(A, B)

syms s;
damp = 0.707;
% damp = 0.595;
wn = 1.13;
lamda1 = -damp*wn + wn*sqrt(1-damp*damp)*1i;
lamda2 = -damp*wn - wn*sqrt(1-damp*damp)*1i;
%   2 times faster
lamda3 = -1.6;
lamda4 = -2.4;
% lamda3 = -2.4 + wn*sqrt(1-damp*damp)*1i;
% lamda4 = -2.4 - wn*sqrt(1-damp*damp)*1i;
%   4 times faster
lamda5 = -3.2;
lamda6 = -4.0;
% lamda5 = -4.0 + wn*sqrt(1-damp*damp)*1i;
% lamda6 = -4.0 - wn*sqrt(1-damp*damp)*1i;
polynomial = (s-lamda1)*(s-lamda2)*(s-lamda3)*(s-lamda4)*(s-lamda5)*(s-lamda6);
pol_cof = double(coeffs(polynomial));

p = [lamda1 lamda2 lamda3 lamda4 lamda5 lamda6];
K11 = place(A,B,p);

Ad = [0   1   0   0   0   0;
      0   0   1   0   0   0;
      0   0   0   1   0   0;
      0   0   0   0   1   0;
      0   0   0   0   0   1;
      -pol_cof(1 : 6)];

syms k11 k12 k13 k14 k15 k16 k21 k22 k23 k24 k25 k26;
K = [k11 k12 k13 k14 k15 k16;
     k21 k22 k23 k24 k25 k26];
Wc = [B A*B A^2*B A^3*B A^4*B A^5*B];
assert(rank(Wc(:, 1:6)) == 6);
CC = Wc(:, 1:6);
CC = [CC(:,1), CC(:,3), CC(:,5), CC(:,2), CC(:,4), CC(:,6)];
C_inv = inv(CC);
d1 = 3;
d2 = 3;
T = [C_inv(d1,:); 
     C_inv(d1,:)*A;
     C_inv(d1,:)*A*A;
     C_inv(d1+d2,:);
     C_inv(d1+d2,:)*A;
     C_inv(d1+d2,:)*A*A];
Aba = T*A/T;
Bba = T*B;
% Aba = round(Aba);
% Bba = round(Bba);
Aba(abs(Aba) < 1e-5) = 0;
Bba(abs(Bba) < 1e-5) = 0;

K_num = solve(Ad([3,6],:) == Aba([3,6],:) - Bba([3,6],:)*K);


% K_num = solve(Ad == Aba-Bba*K);
K_ans = double(struct2array(K_num));
Kba = [K_ans(1:6);
       K_ans(7:12)];
K = Kba * T;

end


