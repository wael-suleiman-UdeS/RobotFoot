function [ePos1, eTheta1, ePos2, eTheta2, DH1, DH2  ] = deltaDeplacementGD(i,L4,L5,LTX,LTZ, q, RightFootTraj, PelvisTraj, LeftFootTraj, TPelvisTraj, TRightTraj, TLeftTraj)


M_RP_1 = [0 1 0 LTX ; 0 0 1 0 ; 1 0 0 LTZ ; 0 0 0 1];
M_PR_1 = [0 0 1 -LTZ ; 1 0 0 -LTX ; 0 1 0 0 ; 0 0 0 1];
M_PR_2_fin = [1 0 0 -LTX ; 0 1 0 0 ; 0 0 1 LTZ ; 0 0 0 1];

MatBaseChange = BaseChange(TLeftTraj(i,3));
RightFootPos = RightFootTraj(i,:)*MatBaseChange;
LeftFootPos = LeftFootTraj(i,:)*MatBaseChange;
PelvisFootPos = PelvisTraj(i,:)*MatBaseChange;

Pe0p = LeftFootPos;
Td1(1,:) = -TPelvisTraj(i,3) + TLeftTraj(i,3);
Td1(2,:) = TPelvisTraj(i,1);
Td1(3,:) = TPelvisTraj(i,2);

Td2 = TRightTraj(i,:)';
Td2(3,:) = Td2(3,:) - TPelvisTraj(i,3);

% Update DH
    DH1 = UpdateDH_GP(L4,L5,q(7:12));
    DH2 = UpdateDH_PD(L4,L5,q(1:6));

    % Update actual position
    TempPe1 = Matrice_Homogene(DH1);
    Pe1 = TempPe1(1:3,4);
    Te1 = UpdateTeGD1(q);

	TempPe2 = Matrice_Homogene(DH2);
    Pe2 = TempPe2(1:3,4);
    Te2 = UpdateTeGD2(q);
	 
    Pe1p = [1 0 0 Pe1(1) ; 0 1 0 Pe1(2) ; 0 0 1 Pe1(3) ; 0 0 0 1];
    Pe1p = M_RP_1*Pe1p;
	Pe1p = Pe1p(1:3,4) + Pe0p';

    % Update desired position
    Pd1p = PelvisFootPos - Pe0p;
	Pd1 = [1 0 0 Pd1p(1) ; 0 1 0 Pd1p(2) ; 0 0 1 Pd1p(3) ; 0 0 0 1];
	Pd1 =  M_PR_1 * Pd1;
    Pd1 = Pd1(1:3,4);

	ePos1 = Pd1-Pe1;
	eTheta1 = Td1-Te1;

	Pd2p = RightFootPos;
	Pd2 = [1 0 0 Pd2p(1)-Pe1p(1) ; 0 1 0 Pd2p(2)-Pe1p(2) ; 0 0 1 Pd2p(3)-Pe1p(3) ; 0 0 0 1];
	Pd2 = Pd2 * M_PR_2_fin;
	Pd2 = Pd2(1:3,4);
	
    ePos2 = Pd2-Pe2;
    eTheta2 = Td2-Te2;

end
