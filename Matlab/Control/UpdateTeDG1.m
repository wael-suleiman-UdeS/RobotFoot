function [ Te ] = UpdateTeDG1( q )

    Te = [0 0 0]';
    %Te(1) = q(2) + q(6);
    %Te(2) = q(3) + q(4) + q(5);
    %Te(3) = q(1);
	Te(1) = q(6);
    Te(2) =  -q(2) - q(3) - q(4);
    Te(3) =  q(1) + q(5);


end
