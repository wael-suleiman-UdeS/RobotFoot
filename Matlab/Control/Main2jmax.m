clc
close all
clear all

% Error Threshold
DistanceThreshold = 0.002;
AngleThreshold = 1;
NbIterationMax = 1;
% Angle
Angle1 = -0.35;
Angle2 = 0.7;
% Longueur
L4 = 0.093;
L5 = 0.093; 
LTX = 0.037; % repère robotfoot C'est LTZ sur Darwin
LTZ = 0.122;  % "                     " LTY
LTY = 0.005;  % "                     " LTX
LF = 0.037;
% Angle de depart
q = [0 Angle1 Angle2 Angle1 0 0 0 0 Angle1 Angle2 Angle1 0];


Td1 = [0; 0; 0];
Td2 = [0; 0; 0];

file_id = fopen('input.txt', 'w');

[Size,dt,T,RightFootTraj,LeftFootTraj,PelvisTraj] = GenerateTrajectory();
FixedFoot = 0 ;  % If FixedFoot = 0 then the right foot is fixed,
                % If FixedFoot = 1 then the left foot is fixed.
           % It should be replace by a function to alternate walking foot

TableQPos = [];
TableQDot = [0 0 0 0 0 0 0 0 0 0 0 0];
TableNbIteration = [];
TableEPos1 = [];
TableEPos2 = [];
TableETheta1 = [];
TableETheta2 = [];



for i = 1:Size

    CalculDone = false;
    NbIteration = 0;

    while ~CalculDone
        NbIteration = NbIteration + 1;
        if  FixedFoot == 0 
            [ePos1, eTheta1, ePos2, eTheta2, DH1, DH2  ] = deltaDeplacementDG(i, L4, L5, LTX, LTZ, q, LeftFootTraj, PelvisTraj, Td1, Td2);
        elseif FixedFoot == 1
            [ePos1, eTheta1, ePos2, eTheta2, DH1, DH2  ] = deltaDeplacementGD(i, L4, L5, LTX, LTZ, q, RightFootTraj, PelvisTraj, Td1, Td2);
        end 
            % Cacul Jacobian
            k = 0.9;
            k2 = 0.1;

            [jacobienne, positions] = Jacobienne(DH1, 1);
            J1 = jacobienne(1:3, 1:6);
            J1inv = pinv2(J1, k2);
            J2 = jacobienne(4:6, 1:6);
            J2inv  = pinv2(J2, k2);

            [jacobienne2, positions2] = Jacobienne(DH2, 1);
            J3 = jacobienne2(1:3, 1:6);
            J3inv = pinv2(J3, k2);
            J4 = jacobienne2(4:6, 1:6);
            J4inv  = pinv2(J4, k2);            

           	% Calcul new motor position
            priorite1 = J1inv * ePos1;
            priorite2 = J2inv * (eTheta1 - (J2 * priorite1));
            priorite3 = J3inv * ePos2;
            priorite4 = J4inv * (eTheta2 - (J4 * priorite3));
            
        if  FixedFoot == 0 
            q(1:6) = q(1:6) + k*(priorite1' + priorite2') ;
            q(7:12)= q(7:12)+ k*(priorite3' + priorite4') ;        
        elseif FixedFoot == 1
            TempQ = q(end:-1:1) ;
            TempQ(1:6) = TempQ(1:6) + k*(priorite1' + priorite2') ;
            TempQ(7:12)= TempQ(7:12)+ k*(priorite3' + priorite4') ;  
            q = TempQ(end:-1:1) ;
        end 
             if  FixedFoot == 0 
                 [ePos1, eTheta1, ePos2, eTheta2, DH1, DH2  ] = deltaDeplacementDG(i, L4, L5, LTX, LTZ, q, LeftFootTraj, PelvisTraj, Td1, Td2);
             elseif FixedFoot == 1
                 [ePos1, eTheta1, ePos2, eTheta2, DH1, DH2  ] = deltaDeplacementGD(i, L4, L5, LTX, LTZ, q, RightFootTraj, PelvisTraj, Td1, Td2);
             end
             
             CalculDone = VerifyError(DistanceThreshold,ePos1,DistanceThreshold,ePos2,AngleThreshold,eTheta1(1),AngleThreshold,eTheta1(2),AngleThreshold,eTheta1(3),AngleThreshold,eTheta2(1),AngleThreshold,eTheta2(2),AngleThreshold,eTheta2(3));
            
            if NbIteration >= NbIterationMax
                CalculDone = true;
            end
            
            if CalculDone 
                displayQ = [q(6) -q(5) q(4) q(3) -q(2) q(1) q(7) q(8) q(9) q(10) -q(11) -q(12)];
                displayQ = displayQ * 180/pi;
                fprintf(file_id,'%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\r\n', displayQ);

                TableQPos = [TableQPos;displayQ] ;
                if i ~= 1
                    TableQDot = [TableQDot; (TableQPos(i,:) - TableQPos(i-1,:))/dt];
                end 
                TableNbIteration = [TableNbIteration; NbIteration];
                TableEPos1 = [TableEPos1; ePos1'];
                TableEPos2 = [TableEPos2; ePos2'];
                TableETheta1 = [TableETheta1; eTheta1'];
                TableETheta2 = [TableETheta2; eTheta2'];
            end

    end
end
fclose(file_id);

figure()
plot(T,TableQPos);
title('TableQPos');
figure()
plot(T,TableQDot);
title('TableQDot');
figure()
plot(T,TableNbIteration);
title('TableNbIteration');
figure()
plot(T,LeftFootTraj);
title('LeftFootTraj');
legend('Position en x', 'Position en y', 'Position en z')
figure()
plot(T,RightFootTraj);
title('RightFootTraj');
legend('Position en x', 'Position en y', 'Position en z')
figure()
plot(T,PelvisTraj);
title('PelvisTraj');
legend('Position en x', 'Position en y', 'Position en z')
figure()
plot(T,TableEPos1);
title('TableEPos1');
figure()
plot(T,TableEPos2);
title('TableEPos2')
figure()
plot(T,TableETheta1);
title('TableETheta1')
figure()
plot(T,TableETheta2);
title('TableETheta2')

EPos1Max = max(TableEPos1);
EPos2Max = max(TableEPos2);
ETheta1Max = max(TableETheta1);
ETheta2Max = max(TableETheta2);
