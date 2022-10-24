function [Gkl] = decoupling_pole(A,B,C,D,pole)
%DECOUPLING_POLE 此处显示有关此函数的摘要
%   此处显示详细说明
G1=ss(A,B,C,D);
G=tf(G1);
[z,p,k]=zpkdata(G(:,:),'v');
for k=1:length(z)
    z1(k)=length(z{k});
end
for k=1:length(p)
    p1(k)=length(p{k});
end
m=size(B,2);
for i=1:m
    for j=1:m
        d1(i,j)=length(p{i,j})-length(z{i,j});
    end
end
for k=1:m
    d(k)=min(d1(k,:))-1;
end
[num1,den1]=tfdata(G(:,:),'v');
syms t;
num = [t];
den=[t];
for i=1:m
    for j=1:m
        num(i,j)=poly2sym(num1{i,j});
    end
end
for i=1:m
    for j=1:m
        den(i,j)=poly2sym(den1{i,j});
    end
end
Gg=num./den;
d;
syms x;
E1=[x,x];
for k=1:m
    E1(k,:)=x^(d(k)+1)*Gg(k,:);
end
E=limit(E1,x,inf);
if det(E)==0
    error('E is singular, decoupling failed!');
end
if det(E)~=0
    disp('E is nonsingular, system can be decoupled!');
end
n=size(A,1);
p=d+ones(1,m);
q(1)=p(1);
for k=1:m-1
    q(k+1)=q(k)+p(k+1);
end
a=[0,q];
for k=1:m
    s=eye(n);
    for i=a(k)+1 : a(k+1)
        s=(A-pole(i)*eye(n))*s;
    end
    F(k,:)=C(k,:)*s;
end
L=inv(E);
K=inv(E)*F;
syms s;
disp('Final transfer funcion');
Gkl=C*inv((s*eye(size(A,1))-A+B*K))*B*L;


end

