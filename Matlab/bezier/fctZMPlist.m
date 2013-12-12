function ZMPlist = fctZMPlist( PA, PD, PasG, PasD )
%ZMPLIST Summary of this function goes here
%   Detailed explanation goes here


    inc = 0;
    for i = 1:2:((length(PasG) +length(PasD)))

         if i-inc <= length(PasD)
            ZMPlist(i,1:2) =  PasD (i-inc, 1:2);
        end

        if (i-inc) <= length(PasG)
            ZMPlist(i+1,1:2)   =  PasG (i-inc, 1:2);
        end

        if (i-inc) == length(PasG) && length(PasG) < length(PasD)
            ZMPlist(i+2,1:2) =  PasD (i-inc+1, 1:2);
        end    

        inc = inc+1;
    end

        ZMPlist = [PA(1:2) ; ZMPlist(2:end-1, 1:2) ; PD(1:2) ] ;

end

