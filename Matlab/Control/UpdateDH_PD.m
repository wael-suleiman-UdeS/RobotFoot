function DH = UpdateDH_PD(L4,L5,q) 
% 
% 	DH = [  0    0     0	  q(1); 
% 	        0   pi/2  0      q(2)+pi/2;
% 	        -L4   0     0      q(3)-pi/2 ;
% 	        -L5   0     0      q(4);
% 	        0   0     0      q(5);
% 	        0    0     0      q(6)+pi/2 ];
         
	DH = [  0       -pi/2   0         q(1) ;
			0   	pi/2    0       q(2)-pi/2 ;
			-L4     0		0         q(3) ;
			-L5     0       0         q(4);
			0       -pi/2   0         q(5);
			0       0       0         q(6) ];
        
      
end
