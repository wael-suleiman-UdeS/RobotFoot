%
%
graphics_toolkit fltk

acx = [0 0];
acy = [0 0];
acz = [0 0];
gyx = [0 0];
gyy = [0 0];
gyz = [0 0];

figacc = figure();
plot3(acx,acy,acz,'xdatasource','acx','ydatasource','acy','zdatasource','acz');
title('Accelerometer');
xlabel('x'); ylabel('y'); zlabel('z');
axis([-1, 1, -1, 1, -1, 1], 'equal');

figgyr = figure();
plot3(gyx,gyy,gyz,'xdatasource','gyx','ydatasource','gyy','zdatasource','gyz');
title('Gyroscope');
xlabel('x'); ylabel('y'); zlabel('z');
axis([-1, 1, -1, 1, -1, 1], 'equal');

while (true)
    eval(["dataraw = [",input('','s'),"];"]);
    acc = dataraw(1:3)/(32768/1.3);
    gyr = dataraw(4:6)/(32768/1.3);

    acx(1) = acc(1);
    acy(1) = acc(2);
    acz(1) = acc(3);
    
    gyx(1) = gyr(1);
    gyy(1) = gyr(2);
    gyz(1) = gyr(3);
    
    refreshdata([figacc, figgyr]);
    drawnow();
    pause(eps);
end

