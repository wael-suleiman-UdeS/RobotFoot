function [ Bx By ] = BezierDegre2( ptA, ptD, StartAngle, EndAngle )
%BEZIERDEGRE2 Summary of this function goes here
%   t varie entre 0 et 1.
%   ptA est le point de départ
%   ptB est le point final
%   ptC est le point intermédiaire
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Bx et By sont les coordonnées de ptA à ptB influencé pas ptC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 %Bx = (1-t).^2 * ptA(1) + 2*t.*(1-t).*ptC(1) + t.^2*ptB(1);
 %By = (1-t).^2 * ptA(2) + 2*t.*(1-t).*ptC(2) + t.^2*ptB(2);
 
 % TODO : Could be a parameter
 distParam = 0.5;
 
 % Find 2 intermediary points to represent starting and ending angle.
 ptB(1) = ptA(1)+distParam*cosd(StartAngle);
 ptB(2) = ptA(2)+distParam*sind(StartAngle);
 
 ptC(1) = ptD(1)-distParam*cosd(EndAngle);
 ptC(2) = ptD(2)-distParam*sind(EndAngle); 
 
 t = 0:0.01:1;
 
 Bx =  ptA(1)*(1-t).^3 + 3*ptB(1)*t.*(1-t).^2 + 3*ptC(1)*t.^2.*(1-t) + ptD(1)*t.^3;
 By =  ptA(2)*(1-t).^3 + 3*ptB(2)*t.*(1-t).^2 + 3*ptC(2)*t.^2.*(1-t) + ptD(2)*t.^3;
 
 figure(1)

 plot(ptA(1),ptA(2),'*' , ptB(1),ptB(2),'*', ptC(1),ptC(2),'*',ptD(1),ptD(2),'*')
 hold on

end

