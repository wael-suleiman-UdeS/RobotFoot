% Filename: Matrice_Homogene.m

function MF = Matrice_Homogene(DH)

MF  = eye(4);
for i=1:size(DH,1)

    A = [ cos(DH(i,4))  -sin(DH(i,4))*cos(DH(i,2))    sin(DH(i,4))*sin(DH(i,2))  DH(i,1)*cos(DH(i,4));
          sin(DH(i,4))   cos(DH(i,4))*cos(DH(i,2))   -cos(DH(i,4))*sin(DH(i,2))  DH(i,1)*sin(DH(i,4));
              0               sin(DH(i,2))                    cos(DH(i,2))                DH(i,3)    ;
              0                     0                               0                        1       ];
MF= MF*A;
end

end

      
      
      
      