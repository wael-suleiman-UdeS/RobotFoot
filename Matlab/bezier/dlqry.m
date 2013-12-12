function [k,s,e] = dlqry(a,b,c,d,q,r)
%DLQRY	Linear quadratic regulator design with output weighting for 
%	discrete-time systems.
%
%	[K,S,E] = DLQRY(A,B,C,D,Q,R)  calculates the optimal feedback gain
%	matrix K such that the feedback law  u[n] = -Kx[n]  minimizes the
%	cost function
%
%		J = Sum {y'Qy + u'Ru}
%
%	subject to the constraint equation:   
%
%		x[n+1] = Ax[n] + Bu[n] 
%		  y[n] = Cx[n] + Du[n]
%                
%	Also returned is S, the steady-state solution to the associated 
%	discrete matrix Riccati equation and the closed loop eigenvalues
%	E = EIG(A-B*K).
%
%	The controller can be formed with DREG.
%
%	See also: DLQR, LQRD, and DREG.

%	Clay M. Thompson  7-23-90
%	Copyright (c) 1986-93 by the MathWorks, Inc.

%error(nargchk(6,6,nargin));
qq = c'*q*c;
rr = r + d'*q*d;
nn = c'*q*d;
[k,s,e] = dlqr(a,b,qq,rr,nn);



