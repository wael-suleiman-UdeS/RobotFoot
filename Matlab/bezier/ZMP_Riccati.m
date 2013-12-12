function [K,f]= ZMP_Riccati(A0,b0,c0,Q,R,N),


[K1,P,E] = dlqry(A0,b0,c0,zeros(size(c0,1),size(b0,2)),Q,R);


K=inv(R+b0'*P*b0)*b0'*P*A0;


A0b0K=eye(size(A0));
f=[];
for i=1:N,
    f=[f,inv(R+b0'*P*b0)*b0'*A0b0K*c0'*Q];
    A0b0K=A0b0K*(A0-b0*K)';
    
end;

return
