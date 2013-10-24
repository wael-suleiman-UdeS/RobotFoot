function DH = UpdateDH_PD(L4,L5,q) 

         
	DH = [  0     -pi/2     0         q(6) ;
			0     +pi/2     0         q(5)+pi/2 ;
			L4     0   	    0         q(4) ;
			L5     0        0         q(3) ;
			0      -pi/2     0         q(2) ;
			0       0       0         q(1) ];
        
      
end
