#include "Trajectory.h"
#include "MotionControl.h"
#include "EigenUtils.h"
#include "DenavitHartenberg.h"

#include "../../ThirdParty/Eigen/Dense"

#include <iostream>
#include <fstream>
#include <cstdlib>
using namespace std;

int main(int argc, char* argv[]) {
	Eigen::Vector2f pointA(0, 0);
	Eigen::Vector2f pointD(3, 0.5);
	Eigen::Vector2f startAngle(0, 0);
	Eigen::Vector2f endAngle(0, 0);


	ofstream myfile;
	std::system("pwd");
	myfile.open ("matrix.txt");

	Trajectory* traj = new Trajectory();
	Eigen::MatrixXf matrix = traj->GenerateWalk(pointA, pointD, startAngle,
			endAngle);

	for(int i = 0; i < matrix.rows(); ++i)
	{
		myfile << matrix.row(i) << endl;
	}

	myfile.close();

	/*
	MotionControl* motion = new MotionControl();
	motion->Walk(matrix);

	Eigen::MatrixXf matrice(3, 6);
	matrice << -0.1747, -0.1747, -0.0874, 0, 0,0,
			0, 0, -0.0319, 0, 0, 0,
			0, 0, 0, 0, 0, 0;


	Eigen::MatrixXf testPseudoInv = EigenUtils::PseudoInverse(matrice);*/


	/*	float a1 = matrix(0,0);
	 float a2 = matrix(0,1);
	 float a3 = matrix(1,0);
	 float a4 = matrix(1,1);
	 float a5 = matrix(2,0);
	 float a6 = matrix(2,1);
	 float a7 = matrix(3,0);
	 float a8 = matrix(3,1);
	 float a9 = matrix(4,0);
	 float a10 = matrix(4,1);
	 float a11 = matrix(5,0);
	 float a12 = matrix(5,1);*/

	int x = 0;

	/*Eigen::VectorXf xPositionsVector(100);
	 Eigen::VectorXf yPositionsVector(100);
	 Eigen::Vector2f pointA(0,0);
	 Eigen::Vector2f pointD(3, 0.5);
	 int startAngle = 0;
	 int endAngle = 0;

	 traj->BezierDegre2(xPositionsVector, yPositionsVector, pointA, pointD, startAngle, endAngle);

	 Eigen::MatrixXf leftTraj(6,2);
	 leftTraj(0,0) = 0;
	 leftTraj(0,1) = 0.1;
	 leftTraj(1,0) = 0.5842;
	 leftTraj(1,1) = 0.1507;
	 leftTraj(2,0) = 1.1767;
	 leftTraj(2,1) = 0.2732;
	 leftTraj(3,0) = 1.7767;
	 leftTraj(3,1) = 0.4212;
	 leftTraj(4,0) = 2.3842;
	 leftTraj(4,1) = 0.5467;
	 leftTraj(5,0) = 2.9995;
	 leftTraj(5,1) = 0.6;

	 Eigen::MatrixXf rightTraj(7,2);
	 rightTraj(0,0) = 0;
	 rightTraj(0,1) = -0.1;
	 rightTraj(1,0) = 0.3090;
	 rightTraj(1,1) = -0.0856;
	 rightTraj(2,0) = 0.9205;
	 rightTraj(2,1) = 0.0101;
	 rightTraj(3,0) = 1.5243;
	 rightTraj(3,1) = 0.1530;
	 rightTraj(4,0) = 2.1205;
	 rightTraj(4,1) = 0.2941;
	 rightTraj(5,0) = 2.7090;
	 rightTraj(5,1) = 0.3864;
	 rightTraj(6,0) = 3.0005;
	 rightTraj(6,1) = 0.4;
	 */
	//Eigen::MatrixXf matrix = traj->ToList(pointA, pointD, leftTraj, rightTraj);
	//float testNorm = leftTraj.col(0).norm();
	//Eigen::MatrixXf matrix2 = traj->MXB(matrix.row(0), matrix.row(2), 0.001);
	//float test = matrix2(0,1);
	//float test1 = matrix2(5,1);
	//float test2 = matrix2(10,1);
	//float test3 = matrix2(15,1);
	//float test4 = matrix2(20,1);
	/*
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
	 */

	//Eigen::MatrixXf  matrix4 = traj->TemporalZMP(matrix, 2, 0.005);
	/*
	 Eigen::VectorXf xInner(xPositionsVector.innerSize());
	 Eigen::VectorXf yInner(yPositionsVector.innerSize());
	 Eigen::VectorXf xOuter(xPositionsVector.innerSize());
	 Eigen::VectorXf yOuter(yPositionsVector.innerSize());*/
	//xPositionsVector and yPositionsVector does not start a 0.0, why?
	//traj->ParallelCurve(xInner,yInner,xOuter,yOuter,xPositionsVector,yPositionsVector,0.1);
	return 0;
}
