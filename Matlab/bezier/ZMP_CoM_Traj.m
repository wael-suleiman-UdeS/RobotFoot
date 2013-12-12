function [Xcom,pk]=ZMP_CoM_Traj(ZMP_ref,zmpx0,zh,R)
%Xcom is the trajectory of center of mass and pk is the ZMP
%of the inverted pendulum model


global T G  % T is the sampling time and G is the Gravitational acceleration 9.8m/s^2

%%%%%%%%%%%%%%%%%%%%%%%% Building the constantes matrices
b0 = [ 0 ; 0 ; T ];
A0 = [  1+G*T^2/zh  T   -G*T^2/zh;
        G*T/zh      1   -G*T/zh;
        0           0       1];
c0 = [0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Predictive control
Q=1;
if nargin<4    
    R=1e-10*eye(size(b0,2));
end

N=1000;
[K,f]= ZMP_Riccati(A0,b0,c0,Q,R,N);

A=A0-b0*K;

X0=[zmpx0;0;zmpx0];


for i=1:N
    ZMP_ref=[ZMP_ref;ZMP_ref(end)];
end

Xk_ant=X0;
Xk_vec=[];

for k=1:length(ZMP_ref)-N,
Xk=A*Xk_ant+b0*f*ZMP_ref(k+1:k+N,1);

Xk_ant=Xk;

Xk_vec=[Xk_vec,Xk];

end;

pk=c0*Xk_vec;
Xcom=Xk_vec(1,:);

return