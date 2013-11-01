function [Size,Dt,T,RightFootTraj,LeftFootTraj,PelvisTraj] = GenerateTrajectory()

Dt = 0.01;
% Time for each following trajectory
%Tf = [1;1;0.2; 0.5;0.2; 0.5;0.5; 0.2;1];
%Tf = [0.3;1;0.3;0.5;1;0.3;0.2;0.5];

% Goal Position for each Trajectory
RightFootPos = [ 0.037,0,0 ;
                 0.037,0,0 ;
                 0.037,0,0 ;
                 0.037,0,0 ;
                 0.037,0,0 ;
                 0.037,0,0 ;
                 0.037,0,0 ;
                 0.037,0,0 ;
                 0.037,0,0 ;
                 0.037,0,0 ;
                 0.037,0,0 ];

LeftFootPos = [  -0.037,0,0 ;
                 -0.037,0,0 ;
                 -0.037,0,0 ;
                 -0.037,0,0 ;
                 -0.037,0,0.01 ;
                 -0.037,-0.08,0.01 ;
                 -0.037,-0.08,0.01 ;
                 -0.037,0.1,0.01 ;
                 -0.037,0.1,0.01 ];
           
 PelvisPos =   [  0.0,0,0.29672 ;
                  0.0,0,0.29672 ;
                  0.05,0,0.29672 ;
                  0.05,0,0.29672 ;
                  0.05,0,0.29672 ;
                  0.05,0,0.29672 ;
                  0.05,0,0.29672 ;
                  0.05,0,0.29672 ;
                  0.05,0,0.29672 ;
                  0.05,0,0.29672 ;
                  0.05,0,0.29672 ];

Tf = [1];

RightFootPos = [ 0.037,0,0 ;
                 0.037,0,0 ];

LeftFootPos = [ -0.037,0,0 ;
                -0.037,0,0 ];

PelvisPos =   [  0.0,0,0.29672 ;
                 -0.05,0.0,0.29672 ];

for i = 1:size(Tf,1)
   RightFootParams(:,(i-1)*3+1) = FunctionParameter(RightFootPos(i,1),RightFootPos(i+1,1),Tf(i));
   RightFootParams(:,(i-1)*3+2) = FunctionParameter(RightFootPos(i,2),RightFootPos(i+1,2),Tf(i));
   RightFootParams(:,(i-1)*3+3) = FunctionParameter(RightFootPos(i,3),RightFootPos(i+1,3),Tf(i));

   LeftFootParams(:,(i-1)*3+1) = FunctionParameter(LeftFootPos(i,1),LeftFootPos(i+1,1),Tf(i));
   LeftFootParams(:,(i-1)*3+2) = FunctionParameter(LeftFootPos(i,2),LeftFootPos(i+1,2),Tf(i));
   LeftFootParams(:,(i-1)*3+3) = FunctionParameter(LeftFootPos(i,3),LeftFootPos(i+1,3),Tf(i));

   PelvisParams(:,(i-1)*3+1) = FunctionParameter(PelvisPos(i,1),PelvisPos(i+1,1),Tf(i));
   PelvisParams(:,(i-1)*3+2) = FunctionParameter(PelvisPos(i,2),PelvisPos(i+1,2),Tf(i));
   PelvisParams(:,(i-1)*3+3) = FunctionParameter(PelvisPos(i,3),PelvisPos(i+1,3),Tf(i));
end

RightFootTraj = [];
LeftFootTraj = [];
PelvisTraj = [];
T = [];
saveT = 0;
for i = 1:size(Tf,1)
   for t = 0:Dt:Tf(i)-Dt
      RightFootTraj = [RightFootTraj; Trajectory(RightFootParams(:,(i-1)*3+1:(i-1)*3+3),t)];
      LeftFootTraj = [LeftFootTraj; Trajectory(LeftFootParams(:,(i-1)*3+1:(i-1)*3+3),t)];
      PelvisTraj = [PelvisTraj; Trajectory(PelvisParams(:,(i-1)*3+1:(i-1)*3+3),t)];
   
      T = [T;saveT];
      saveT = saveT+Dt;
   end
end

Size = size(T,1);

end

