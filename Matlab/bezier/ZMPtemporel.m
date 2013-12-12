function DeplacementTemporel = ZMPtemporel( PA, Tp, Tech, ZMPlist)
%ZMPTEMPOREL Summary of this function goes here
%   Detailed explanation goes here

Tpn = 0:Tech:Tp;
% Temps 
Ts =0.2*Tp ;  %sec
Tsn = 0:Tech:Ts;

    TrajX = [];
    TrajY = [];

        Tempstotal = (length(ZMPlist)-1)*(Ts+Tp);
        
        tempDeplacement = [PA(1) PA(2) 0];
      
    for j = 1:1: length(ZMPlist)-1

        [DroiteX DroiteY ] = mxb(Tsn/Ts, ZMPlist(j,1:2), ZMPlist(j+1,1:2));
        DeplacementZMP = [DroiteX(2:end) ; DroiteY(2:end) ; Tsn(2:end)]';
        
        Xattend(1:Tp/Tech) = ZMPlist(j+1,1);
        Yattend(1:Tp/Tech) = ZMPlist(j+1,2);
        DeplacementPied = [ Xattend ; Yattend ; Tpn(2:end)]';
        
        Deplacement     = [DeplacementZMP ; DeplacementPied];
        tempDeplacement  = [tempDeplacement ; Deplacement];
  
    end
 
 Ttn = [0:Tech:Tempstotal]'  ;
DeplacementTemporel = [tempDeplacement(1:end,1), tempDeplacement(1:end,2), Ttn ];

end

