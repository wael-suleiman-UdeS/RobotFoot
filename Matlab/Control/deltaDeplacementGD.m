function [ePos1, eTheta1, ePos2, eTheta2, DH1, DH2  ] = deltaDeplacementGD(i,L4,L5,LTX,LTZ, q, LeftFootTraj, PelvisTraj, Td1, Td2)


M_RP_1 = [1 0 0 LTX ; 0 0 1 0 ; 0 1 0 LTZ ; 0 0 0 1];
M_PR_1 = [0 0 1 0 ; 1 0 0 0 ; 0 1 0 0 ; 0 0 0 1];
M_PR_1_fin = [1 0 0 LTX ; 0 1 0 0 ; 0 0 1 -LTZ ; 0 0 0 1];
M_RP_2 = [1 0 0 -LTX ; 0 1 0 0 ; 0 0 1 -LTZ ; 0 0 0 1 ];
M_PR_2_fin = [1 0 0 LTX ; 0 1 0 0 ; 0 0 1 LTZ ; 0 0 0 1];

% Update DH
    q = q(end:-1:1)
    DH1 = UpdateDH_GP(L4,L5,q(1:6));
    DH2 = UpdateDH_PD(L4,L5,q(7:12));

    % Update actual position
    TempPe1 = Matrice_Homogene(DH1);
    Pe1 = TempPe1(1:3,4);
    % Should be Updated
    %Te1 = UpdateTe1(q);

	TempPe2 = Matrice_Homogene(DH2);
    Pe2 = TempPe2(1:3,4);
    % Should be Updated
    %Te2 = UpdateTe2(q);
	 
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
    %eTheta2 = Td2-Te2;

end
