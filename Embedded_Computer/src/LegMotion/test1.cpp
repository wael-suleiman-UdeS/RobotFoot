#include "Trajectory.h"

#include "../ThirdParty/Eigen/Dense"

int main(int argc, char* argv[])
{
        Trajectory* traj = new Trajectory();

	Eigen::VectorXf xPositionsVector(100);
	Eigen::VectorXf yPositionsVector(100);
	Eigen::Vector2f pointA(0,0);
	Eigen::Vector2f pointD(3, 0);
	int startAngle = 0;
	int endAngle = 0;

	traj->BezierDegre2(xPositionsVector, yPositionsVector, pointA, pointD, startAngle, endAngle);

	Eigen::MatrixXf leftTraj(6,2);
 	leftTraj(0,0) = 0;
	leftTraj(0,1) = 0.1;
	leftTraj(1,0) = 0.6;
	leftTraj(1,1) = 0.1;
	leftTraj(2,0) = 1.23;
	leftTraj(2,1) = 0.1;
	leftTraj(3,0) = 1.83;	
	leftTraj(3,1) = 0.1;
	leftTraj(4,0) = 2.49;
	leftTraj(4,1) = 0.1;
	leftTraj(5,0) = 3.0;
	leftTraj(5,1) = 0.1;

	Eigen::MatrixXf rightTraj(7,2);
 	rightTraj(0,0) = 0;
	rightTraj(0,1) = -0.1;
	rightTraj(1,0) = 0.30;
	rightTraj(1,1) = -0.1;
	rightTraj(2,0) = 0.93;
	rightTraj(2,1) = -0.1;
	rightTraj(3,0) = 1.53;	
	rightTraj(3,1) = -0.1;
	rightTraj(4,0) = 2.16;
	rightTraj(4,1) = -0.1;
	rightTraj(5,0) = 2.79;
	rightTraj(5,1) = -0.1;
	rightTraj(6,0) = 3.000;
	rightTraj(6,1) = -0.1;


	Eigen::MatrixXf matrix = traj->ToList(pointA, pointD, leftTraj, rightTraj);

	float testNorm = leftTraj.col(0).norm();

	Eigen::MatrixXf matrix2 = traj->MXB(matrix.row(0), matrix.row(2), 0.001);

	float test = matrix2(0,1);
	float test1 = matrix2(5,1);
	float test2 = matrix2(10,1);
	float test3 = matrix2(15,1);
	float test4 = matrix2(20,1);

	Eigen::MatrixXf matrix3 = traj->SpatialZMP(pointA, pointD, leftTraj, rightTraj, 0.001);

	float test5 = matrix3(0,1);
	float test6 = matrix3(500,1);
	float test7 = matrix3(1000,1);
	float test8 = matrix3(1500,1);
	float test9 = matrix3(2000,1);
	float test10 = matrix3(3000,1);
	float test11 = matrix3(4000,1);
	float test12 = matrix3(6000,1);

	float test13 = matrix3(0,0);
	float test14 = matrix3(500,0);
	float test15 = matrix3(1000,0);
	float test16 = matrix3(1500,0);
	float test17 = matrix3(2000,0);
	float test18 = matrix3(3000,0);
	float test19 = matrix3(4000,0);
	float test20 = matrix3(6000,0);

	float test21 = matrix3(1999,1);

	Eigen::MatrixXf  matrix4 = traj->TemporalZMP(pointA, pointD, leftTraj, rightTraj, 2, 0.005);
	

	return 0;
}
