function [ matrice_inverse ] = pinv2( matrice, k )

etape1 = matrice*transpose(matrice);
etape2 = k^2*eye(size(matrice,1));
etape3 = (etape1 + etape2);
matrice_inverse = transpose(matrice)/etape3;

%matrice_inverse = matrice'*inv((matrice*matrice'+k^2*eye(size(matrice, 2))));

end

