clc
close all
clear all

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

M_RP_1 = [0 1 0 -LTX ; 0 0 1 0 ; 1 0 0 LTZ ; 0 0 0 1];
M_PR_1 = [0 0 1 0 ; 1 0 0 0 ; 0 1 0 0 ; 0 0 0 1];
M_PR_1_fin = [1 0 0 LTX ; 0 1 0 0 ; 0 0 1 -LTZ ; 0 0 0 1];
M_RP_2 = [1 0 0 -LTX ; 0 1 0 0 ; 0 0 1 -LTZ ; 0 0 0 1 ];
M_PR_2_fin = [1 0 0 LTX ; 0 1 0 0 ; 0 0 1 LTZ ; 0 0 0 1];

Td1 = [0; 0; 0];
Td2 = [0; 0; 0];

file_id = fopen('input.txt', 'w');

[Size,dt,T,RightFootTraj,LeftFootTraj,PelvisTraj] = GenerateTrajectory();

for i = 1:Size

    % Update DH
    DH1 = UpdateDH1(L4,L5,q);
    DH2 = UpdateDH2(L4,L5,q(7:12));

    % Update actual position
    TempPe1 = Matrice_Homogene(DH1);
    Pe1 = TempPe1(1:3,4);
    Te1 = UpdateTe1(q);

	 TempPe2 = Matrice_Homogene(DH2);
    Pe2 = TempPe2(1:3,4);
    Te2 = UpdateTe2(q);
	 
    Pe1p = [1 0 0 Pe1(1) ; 0 1 0 Pe1(2) ; 0 0 1 Pe1(3) ; 0 0 0 1];
	 Pe1p = M_RP_1*Pe1p;
	 Pe1p = Pe1p(1:3,4);

	 Pe2p = [1 0 0 Pe2(1) ; 0 1 0 Pe2(2) ; 0 0 1 Pe2(3) ; 0 0 0 1];
	 Pe2p = M_RP_2*Pe2p;
	 Pe2p = Pe1p + Pe2p(1:3,4);

	 Pe2 = [1 0 0 Pe2p(1)-Pe1p(1) ; 0 1 0 Pe2p(2)-Pe1p(2) ; 0 0 1 Pe2p(3)-Pe1p(3) ; 0 0 0 1];
	 Pe2 = Pe2 * M_PR_2_fin;
	 Pe2 = Pe2(1:3,4);

    % Update desired position
    Pd1p = PelvisTraj(i,:);
	 Pd1 = [1 0 0 Pd1p(1) ; 0 1 0 Pd1p(2) ; 0 0 1 Pd1p(3) ; 0 0 0 1];
	 Pd1 =  M_PR_1 * Pd1 * M_PR_1_fin;
	 Pd1 = Pd1(1:3,4);

	 ePos1 = Pd1-Pe1;
	 eTheta1 = Td1-Te1;

	 Pd2p = LeftFootTraj(i,:); 
	 Pd2 = [1 0 0 Pd2p(1)-Pe1p(1) ; 0 1 0 Pd2p(2)-Pe1p(2) ; 0 0 1 Pd2p(3)-Pe1p(3) ; 0 0 0 1];
	 Pd2 = Pd2 * M_PR_2_fin;
	 Pd2 = Pd2(1:3,4);
	 
    ePos2 = Pd2-Pe2;
    eTheta2 = Td2-Te2;

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

    % 	% Calcul new motor position
    priorite1 = J1inv * ePos1;
    priorite2 = J2inv * (eTheta1 - (J2 * priorite1));
    priorite3 = J3inv * ePos2;
    priorite4 = J4inv * (eTheta2 - (J4 * priorite3));

    q(1:6) = q(1:6) + k*(priorite1' + priorite2') ;
    q(7:12)= q(7:12)+ k*(priorite3' + priorite4') ;


    displayQ = [q(6) -q(5) q(4) q(3) -q(2) q(1) q(7) q(8) q(9) q(10) -q(11) -q(12)];
    displayQ = displayQ * 180/pi
    fprintf(file_id,'%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\r\n', displayQ);

end

fclose(file_id);


