function [ Trajectoire ] = ZMPspatial( ZMPlist )
%ZMPSPATIAL Summary of this function goes here
% La fonction retourne la trajectoire de référence du ZMP
%   ZMPlist : list débutant par le point Initial "PA" suivit en alternance des
%   PasD et PasG (un saute le premier PasD, et le dernier (PasD ou PasG), se terminant par le point Final "PD"
% Trajectoire = Tableau de 2 colonnes (x,y) contenant toute la trajectoire
%   par interpolation des points de "ZMPlist".


    TrajX = [];
    TrajY = [];
    n = 0: 0.001 : 1;

   
    for j = 1:1: length(ZMPlist)-1
        
        [DroiteX DroiteY ] = mxb( n, ZMPlist(j,1:2), ZMPlist(j+1,1:2));
        TrajX = [TrajX DroiteX];
        TrajY = [TrajY DroiteY];
        Trajectoire = [TrajX ; TrajY]';

    end
end

