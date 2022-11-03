function [K11,K] = pole_placement(A, B, p)

syms s;

% lamda1 = -1.25 + 1.875*1i;
% lamda2 = -1.25 - 1.875*1i;
% lamda3 = -5;
% lamda4 = -5.375;
% lamda5 = -5.75;
% lamda6 = -6.25;
% 
% lamda1 = -5 + 1.875*1i;
% lamda2 = -5 - 1.875*1i;
% lamda3 = -20;
% lamda4 = -21.5;
% lamda5 = -23;
% lamda6 = -25;

polynomial = (s-p(1))*(s-p(2))*(s-p(3))*(s-p(4))*(s-p(5))*(s-p(6));
pol_cof = double(coeffs(polynomial));

K11 = place(A,B,p);
% K11=acker(A,B,p);

dim=size(B,2);

if dim==2

    K=zeros(2,6);

    Ad = [0   1   0   0   0   0;
          0   0   1   0   0   0;
          0   0   0   1   0   0;
          0   0   0   0   1   0;
          0   0   0   0   0   1;
          -pol_cof(1 : 6)];
      
    Ad = [0 1 0           0 0 0;
          0 0 1           0 0 0;
          -pol_cof(1:3)   0 0 0;
          0 0 0           0 1 0;
          0 0 0           0 0 1;
          0 0 0           -pol_cof(4:6)];
    
    syms k11 k12 k13 k14 k15 k16 k21 k22 k23 k24 k25 k26;
    K = [k11 k12 k13 k14 k15 k16;
         k21 k22 k23 k24 k25 k26];
    Wc = [B A*B A^2*B A^3*B A^4*B A^5*B];
%     assert(rank(Wc(:, 1:6)) == 6);
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
    
    K_num = solve(Ad == Aba-Bba*K);
    K_ans = double(struct2array(K_num));
    Kba = [K_ans(1:6);
           K_ans(7:12)];
    K = Kba * T;

elseif dim==3

    Ad = [0   1   0   0   0   0;
          0   0   1   0   0   0;
          0   0   0   1   0   0;
          0   0   0   0   1   0;
          0   0   0   0   0   1;
          -pol_cof(1 : 6)];
      
    Ad = [0 1 0           0 0 0;
          0 0 1           0 0 0;
          -pol_cof(1:3)   0 0 0;
          0 0 0           0 1 0;
          0 0 0           0 0 1;
          0 0 0           -pol_cof(4:6)];
    
    syms k11 k12 k13 k14 k15 k16 k21 k22 k23 k24 k25 k26 k31 k32 k33 k34 k35 k36;
    K = [k11 k12 k13 k14 k15 k16;
         k21 k22 k23 k24 k25 k26;
         k31 k32 k33 k34 k35 k36];
    Wc = [B A*B A^2*B A^3*B A^4*B A^5*B];
%     assert(rank(Wc(:, 1:6)) == 6);
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
    if rank(T)<6
        K=place(A,B,p);
        return;
    end
    Aba = T*A/T;
    Bba = T*B;
    Aba(abs(Aba) < 1e-5) = 0;
    Bba(abs(Bba) < 1e-5) = 0;
    
    K_num = solve(Ad == Aba-Bba*K);
    K_ans = double(struct2array(K_num));
    Kba = [K_ans(1:6);
           K_ans(7:12);
           K_ans(13:18)];
    K = Kba * T;

end

end


