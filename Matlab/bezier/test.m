clear all
clc
close all

global T G

G = 9.805 ;

% Starting point
PA = [0 0];
% Ending point
PD = [1 1.3];

% Starting angle
StartingAngle = 90;
% Ending angle
EndingAngle = 0;

% Temps d'Èchantillonnage
Tech = 0.005; %sec
T = Tech ;

% Temps d'un pas
Tp = 0.3;  %sec

zh = 0.3; % ~ 30 cm ?!?

% Calcul bezier curve
[ResultX,ResultY] = BezierDegre2( PA, PD, StartingAngle, EndingAngle );

% Generate parallel curve
dLeg=0.1;
make_plot=0;
flag1=0;
[x_inner, y_inner, x_outer, y_outer, R, unv, concavity, overlap]=parallel_curve(ResultX, ResultY, dLeg, make_plot, flag1);

% A step distance
dStep = 0.1;
dStepSqrt = dStep*dStep;
CurLStepPos = [ PA(1)-dLeg*sind(StartingAngle), PA(2)+dLeg*cosd(StartingAngle) ];
CurRStepPos = [ PA(1)+dLeg*sind(StartingAngle), PA(2)-dLeg*cosd(StartingAngle) ];

trajL = CurLStepPos;
trajR = CurRStepPos;

for i = 1:length(x_inner)

   distL = (x_inner(i)-CurLStepPos(1))^2 + (y_inner(i)-CurLStepPos(2))^2;
   distR = (x_outer(i)-CurRStepPos(1))^2 + (y_outer(i)-CurRStepPos(2))^2;
   
   if( distL>=dStepSqrt || distR>=dStepSqrt)
   
      CurLStepPos = [x_inner(i) y_inner(i)];
      CurRStepPos = [x_outer(i) y_outer(i)];
      trajL = [ trajL; CurLStepPos ];
      trajR = [ trajR; CurRStepPos ];
   end

end

if( x_inner(end) ~= trajL(end,1) || y_inner(end) ~= trajL(end,2) )
  trajL = [ trajL; [x_inner(end) y_inner(end)] ];
end

if( x_outer(end) ~= trajR(end,1) || y_outer(end) ~= trajR(end,2) )
   trajR = [ trajR; [x_outer(end) y_outer(end)] ];
end

PasG = trajL(1,:);
PasD = trajR(1,:);
n = 1;

for i = 2:(length(trajL)-1)
  if mod(i,2) == 0
    PasD = [ PasD; trajR(i,:) ];
  else
    PasG = [ PasG; trajL(i,:) ];
  end
  n = i;
end

n = n+1;
PasG = [ PasG; trajL(n,:) ];
PasD = [ PasD; trajR(n,:) ];



figure(1)
hold on 
plot(ResultX,ResultY, 'r')
%plot(trajR(:,1),trajR(:,2),'o')
plot(PasG(:,1),PasG(:,2),'o')
plot(PasD(:,1),PasD(:,2),'ko')
axis([-0.2 1.1 -0.1 1.5])
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    DETERMINATION DES ZMPref
% boucle qui relie les points de chaque ZMP
%Pour cette partie, il faut conna√Ætre la trajectoire √† emprunt√©, le point
%de d√©part, finale et le point de pression de chaque pas (pasG et pasD)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ZMPlist = fctZMPlist( PA, PD, PasG, PasD );
%%%%%%%%%%%%%%%% DANS LE DOMAINE SPATIALE %%%%%%%%%%%%%%%%%%%%%%%%%%%%

TrajectoireSpatiale = ZMPspatial(ZMPlist);

%%%%%%%%%%%%%%%%%%%%% DANS LE DOMAINE TEMPOREL %%%%%%%%%%%%%%%%%%%%%%%

DeplacementTemporel = ZMPtemporel( PA, Tp, Tech, ZMPlist );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    DETERMINATION de la trajectoire du centre masse
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


[Xcom,pk]   = ZMP_CoM_Traj(DeplacementTemporel(1:end, 1),0,zh);
[Xcom1,pk1] = ZMP_CoM_Traj(DeplacementTemporel(1:end, 2),0,zh);


%%%%%%%%%%%% GRAPHIQUE ! %%%%%%%%%%%%%
figure(1)
plot (TrajectoireSpatiale(1:end, 1), TrajectoireSpatiale(1:end,2), Xcom,Xcom1,'g')
xlabel ('dÈplacement x(m)')
ylabel ('dÈplacement y(m)')
legend('P0', 'P1 ','P2 ','P3', 'Courbe de BÈzier', 'Trace de pied gauche','Trace de pied droit', 'Trajectoire ZMP','Trajectoire Com')
figure(3)
plot(DeplacementTemporel(1:end,3) ,DeplacementTemporel(1:end,1),DeplacementTemporel(1:end,3), Xcom,DeplacementTemporel(1:end,3),pk )
title('ZMP trajectory')
xlabel('temps(s)')
ylabel('x(m)')
%legend('ZMP ref')

figure(4)
plot(DeplacementTemporel(1:end,3) ,DeplacementTemporel(1:end,2),DeplacementTemporel(1:end,3),Xcom1,DeplacementTemporel(1:end,3),pk1 )
title('ZMP trajectory')
xlabel('temps(s)')
ylabel('y(m)')
%legend('ZMP ref')


 