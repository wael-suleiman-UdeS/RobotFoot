function [jacob, P] = Jacobienne(DH, matriceDeRetour)
%matriceDeRetour = 1: Jacobienne complete
%matriceDeRetour = 2: Jacobienne position
%matriceDeRetour = 3: Jacobienne rotation

sizeDH = 1:size(DH,2);
A01 = Matrice_Homogene(DH(1,sizeDH));
A12 = Matrice_Homogene(DH(2,sizeDH));
A23 = Matrice_Homogene(DH(3,sizeDH));
A34 = Matrice_Homogene(DH(4,sizeDH));
A45 = Matrice_Homogene(DH(5,sizeDH));
A56 = Matrice_Homogene(DH(6,sizeDH));

%Should change to?
%A56 = Matrice_Homogene(DH(6,:));

A01 = A01;
A02 = A01*A12;
A03 = A02*A23;
A04 = A03*A34;
A05 = A04*A45;
A06 = A05*A56;

Z(1:3, 1) = [0 ; 0; 1];
Z(1:3, 2) = A01(1:3, 3);
Z(1:3, 3) = A02(1:3, 3);
Z(1:3, 4) = A03(1:3, 3);
Z(1:3, 5) = A04(1:3, 3);
Z(1:3, 6) = A05(1:3, 3);

P(1:3, 1) = [0;0;0];
P(1:3, 2) = A01(1:3, 4);
P(1:3, 3) = A02(1:3, 4);
P(1:3, 4) = A03(1:3, 4);
P(1:3, 5) = A04(1:3, 4);
P(1:3, 6) = A05(1:3, 4);
P(1:3, 7) = A06(1:3, 4);

J = [cross(Z(1:3, 1),(P(1:3, 7)-P(1:3, 1))) cross(Z(1:3, 2), (P(1:3, 7)-P(1:3, 2))) cross(Z(1:3, 3), (P(1:3, 7)-P(1:3, 3))) cross(Z(1:3, 4), (P(1:3, 7)-P(1:3, 4))) cross(Z(1:3, 5), (P(1:3, 7)-P(1:3, 5))) cross(Z(1:3, 6), (P(1:3, 7)-P(1:3, 6)));
     Z(1:3, 1) Z(1:3, 2) Z(1:3, 3) Z(1:3, 4) Z(1:3, 5) Z(1:3, 6)];
 
if(matriceDeRetour == 3)
    jacob = J(4:6, 1:6);
elseif(matriceDeRetour == 2)
    jacob = J(1:3, 1:6);
else
    jacob = J;
end
