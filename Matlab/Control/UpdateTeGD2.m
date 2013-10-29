function [ Te ] = UpdateTeDG2( q )

    Te = [0 0 0]';

	Te(1) = -q(4) - q(3) - q(2);
    Te(2) =  q(5) + q(1);
    Te(3) =  q(6);


end
