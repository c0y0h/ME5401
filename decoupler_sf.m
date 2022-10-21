function [K,F] = decoupler_sf(A,B,C)
%DECOUPLER 此处显示有关此函数的摘要
%   此处显示详细说明
m=size(A,1);
r=size(B,2);
n=size(C,1);
% syms s;
% G = C/(s*eye(m)-A)*B;

sigma=zeros(n,1);
for i=1:n
    count=1;
    for j=1:m
        if C(i,:)*(A^(count-1))*B ~= 0
            break;
        else
            count=count+1;
        end
    end
    if count == m+1
        sigma(i)=m;
    else
        sigma(i)=count;
    end
end

Bstar=zeros(n,r);
Cstar=zeros(n,m);
for i=1:n
    Bstar(i,:)=C(i,:)*(A^(sigma(i)-1))*B;
    Cstar(i,:)=C(i,:)* A^(sigma(i));
end
F=inv(Bstar);
% K=Bstar\Cstar;

% lamda=[-1;
%        -2];
lamda=[-0.8;
       -0.9;
       -1.6;
       -1.7;
       -3.2;
       -3.3];

% polinomial=[(A-eye(m)*lamda(1))*(A-eye(m)*lamda(2));
%             (A-eye(m)*lamda(3))*(A-eye(m)*lamda(4))];
% syms s;
% A_d = (s-lamda(1))*(s-lamda(2));
% A_d_den=double(fliplr(coeffs(A_d)));
% fA1 = A^2+A_d_den(2)*A+A_d_den(3)*eye(m);
% A_d = (s-lamda(3))*(s-lamda(4));
% A_d_den=double(fliplr(coeffs(A_d)));
% fA2 = A^2+A_d_den(2)*A+A_d_den(3)*eye(m);

% fA1=A-eye(m)*lamda(1);
% fA2=A-eye(m)*lamda(2);

% fA1=A^2 - (lamda(1)+lamda(2))*A + eye(m)*lamda(1)*lamda(2);
% fA2=A^2 - (lamda(3)+lamda(4))*A + eye(m)*lamda(3)*lamda(4);
fA1 = A^3 - (lamda(1)+lamda(2)+lamda(3))*A^2 + ...
    (lamda(1)*lamda(2)+lamda(1)*lamda(3)+lamda(2)*lamda(3))*A -...
    lamda(1)*lamda(2)*lamda(3);
fA2 = A^3 - (lamda(4)+lamda(5)+lamda(6))*A^2 + ...
    (lamda(4)*lamda(5)+lamda(4)*lamda(6)+lamda(5)*lamda(6))*A -...
    lamda(4)*lamda(5)*lamda(6);

Csstar=zeros(n,m);
% for i=1:n
% %     Csstar(i,:)=C(i,:)*fA;
%     Csstar(i,:)=C(i,:)*polinomial(i);
% end
Csstar(1,:)=C(1,:)*fA1;
Csstar(2,:)=C(2,:)*fA2;
K=F*Csstar;

end

