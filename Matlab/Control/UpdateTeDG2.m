function [ Te ] = UpdateTeDG2( q )

    Te = [0 0 0]';

	Te(1) = -q(9) - q(10) - q(11);
    Te(2) =  q(8) + q(12);
    Te(3) =  q(7);


end
