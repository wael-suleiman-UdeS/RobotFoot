function [Tf, FixedFoot, RightFootPos, LeftFootPos, PelvisPos, TRightPos, TLeftPos, TPelvisPos] = MouvementTest()
Tf = [1];
FixedFoot = [1];

RightFootPos = [ -0.037,0,0
                 -0.037,0,0 ];

LeftFootPos = [ 0.037,0,0
                0.037,0,0 ];

PelvisPos =   [  0.0,0,0.29672
                 0.0,-0.04,0.29672 ];

TRightPos = [   0,0,3.14
                0,0,3.14 ];

TLeftPos = [    0,0,3.14 
                0,0,3.14 ];

TPelvisPos = [  0,0,3.14 
                0,0,3.14 ];
end

