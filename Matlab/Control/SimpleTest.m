% Generate Traj for mouvement
Dt = 0.01;
Tf = [1];

TrajPos(1,:) = [ 0,0,-0.17472] ;
%TrajPos(1,:) = [ 0.17472,0,0] ;
TrajPos(2,:) = TrajPos(1,:) + [0.0,-0.05,0.0];
Td = [0,0,0];

for i = 1:size(Tf,1)
   TrajParams(:,(i-1)*3+1) = FunctionParameter(TrajPos(i,1),TrajPos(i+1,1),Tf(i));
   TrajParams(:,(i-1)*3+2) = FunctionParameter(TrajPos(i,2),TrajPos(i+1,2),Tf(i));
   TrajParams(:,(i-1)*3+3) = FunctionParameter(TrajPos(i,3),TrajPos(i+1,3),Tf(i));
end

Traj = [];
T = [];
saveT = 0;
for i = 1:size(Tf,1)
   for t = 0:Dt:Tf(i)-Dt
      Traj = [Traj; Trajectory(TrajParams(:,(i-1)*3+1:(i-1)*3+3),t)];
   
      T = [T;saveT];
      saveT = saveT+Dt;
   end
end

Size = size(T,1);

L4 = 0.093;
L5 = 0.093; 
Angle1 = 0.35;
Angle2 = -0.7;

q = [0 Angle1 Angle2 Angle1 0 0 0 0 -Angle1 -Angle2 -Angle1 0];
displayQ = q * 180/pi

for i = 1:Size

    %DH = UpdateDH_DP(L4,L5,q(1:6));
    DH = UpdateDH_PG(L4,L5,q(7:12));

    TempPe = Matrice_Homogene(DH);
    Pe = TempPe(1:3,4);
    %Te = UpdateTeDG1(q);
    Te = UpdateTeDG2(q);

    ePos = Traj(i,:)'-Pe;
    eTheta = Td'-Te;

    % Cacul Jacobian
    k = 0.9;
    k2 = 0.1;
    
    [jacobienne, positions] = Jacobienne(DH, 1);
    J1 = jacobienne(1:3, 1:6);
    J1inv = pinv2(J1, k2);
    J2 = jacobienne(4:6, 1:6);
    J2inv  = pinv2(J2, k2);
    
    % Calcul new motor position
    priorite1 = J1inv * ePos;
    priorite2 = J2inv * (eTheta - (J2 * priorite1));
    
    q(7:12) = q(7:12) + k*(priorite1' + priorite2') ;
    displayQ = q * 180/pi;
    
    Traj(i,:)'
    Pe
    Te
    ePos
    eTheta
    displayQ

end
