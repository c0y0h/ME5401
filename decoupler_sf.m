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

damp = 0.707;
wn = 1.13;
lamda1 = -damp*wn + wn*sqrt(1-damp*damp)*1i;
lamda2 = -damp*wn - wn*sqrt(1-damp*damp)*1i;
lamda3 = -1.6;
lamda4 = -1.7;

fA1 = (A-lamda1*eye(m))*(A-lamda2*eye(m));
fA2 = (A-lamda3*eye(m))*(A-lamda4*eye(m));

Csstar=zeros(n,m);
% for i=1:n
% %     Csstar(i,:)=C(i,:)*fA;
%     Csstar(i,:)=C(i,:)*polinomial(i);
% end
Csstar(1,:)=C(1,:)*fA1;
Csstar(2,:)=C(2,:)*fA2;
K=F*Csstar;


end

