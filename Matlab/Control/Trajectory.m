function traj = Trajectory(param, t)

 traj(1,1) = param(1,1).*t^3+param(2,1).*t^2+param(3,1).*t+param(4,1);
 traj(1,2) = param(1,2).*t^3+param(2,2).*t^2+param(3,2).*t+param(4,2);
 traj(1,3) = param(1,3).*t^3+param(2,3).*t^2+param(3,3).*t+param(4,3);

end
