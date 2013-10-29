function [ Te ] = UpdateTeDG1( q )

    Te = [0 0 0]';
	Te(1) = q(7);
    Te(2) =  -q(11) - q(10) - q(9);
    Te(3) =  q(12) + q(8);


end
