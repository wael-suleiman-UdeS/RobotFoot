function [Size,Dt,T,RightFootTraj,LeftFootTraj,PelvisTraj,FixedFootTraj,TRightTraj,TLeftTraj,TPelvisTraj] = GenerateTrajectory()
% Genere une suite dans trajectoire avec un interpolation cubique entre chacun des points.

Dt = 0.01;

% Choose the trajectory
[Tf, FixedFoot, RightFootPos, LeftFootPos, PelvisPos, TRightPos, TLeftPos, TPelvisPos] = MouvementWalk();

%Parametre de la function cubique pour chacune des trajectoires
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
   
   TRightParams(:,(i-1)*3+1) = FunctionParameter(TRightPos(i,1),TRightPos(i+1,1),Tf(i));
   TRightParams(:,(i-1)*3+2) = FunctionParameter(TRightPos(i,2),TRightPos(i+1,2),Tf(i));
   TRightParams(:,(i-1)*3+3) = FunctionParameter(TRightPos(i,3),TRightPos(i+1,3),Tf(i));
   
   TLeftParams(:,(i-1)*3+1) = FunctionParameter(TLeftPos(i,1),TLeftPos(i+1,1),Tf(i));
   TLeftParams(:,(i-1)*3+2) = FunctionParameter(TLeftPos(i,2),TLeftPos(i+1,2),Tf(i));
   TLeftParams(:,(i-1)*3+3) = FunctionParameter(TLeftPos(i,3),TLeftPos(i+1,3),Tf(i));

   TPelvisParams(:,(i-1)*3+1) = FunctionParameter(TPelvisPos(i,1),TPelvisPos(i+1,1),Tf(i));
   TPelvisParams(:,(i-1)*3+2) = FunctionParameter(TPelvisPos(i,2),TPelvisPos(i+1,2),Tf(i));
   TPelvisParams(:,(i-1)*3+3) = FunctionParameter(TPelvisPos(i,3),TPelvisPos(i+1,3),Tf(i));
end

RightFootTraj = [];
LeftFootTraj = [];
PelvisTraj = [];
FixedFootTraj = [];
TRightTraj = [];
TLeftTraj = [];
TPelvisTraj = [];
T = [];
saveT = 0;

% Genere la trajectoire.
for i = 1:size(Tf,1)
   for t = 0:Dt:Tf(i)-Dt
      RightFootTraj = [RightFootTraj; Trajectory(RightFootParams(:,(i-1)*3+1:(i-1)*3+3),t)];
      LeftFootTraj = [LeftFootTraj; Trajectory(LeftFootParams(:,(i-1)*3+1:(i-1)*3+3),t)];
      PelvisTraj = [PelvisTraj; Trajectory(PelvisParams(:,(i-1)*3+1:(i-1)*3+3),t)];
      
      TRightTraj = [TRightTraj; Trajectory(TRightParams(:,(i-1)*3+1:(i-1)*3+3),t)];
      TLeftTraj = [TLeftTraj; Trajectory(TLeftParams(:,(i-1)*3+1:(i-1)*3+3),t)];
      TPelvisTraj = [TPelvisTraj; Trajectory(TPelvisParams(:,(i-1)*3+1:(i-1)*3+3),t)];
   
      T = [T;saveT];
      FixedFootTraj = [FixedFootTraj; FixedFoot(i)];
      saveT = saveT+Dt;
   end
end

Size = size(T,1);

end

