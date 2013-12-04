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
	Eigen::Vector2f pointD(1, 1);
	//Eigen::Vector2f pointD(1, 1.3);
	Eigen::Vector2f startAngle(0, 0);
	//Eigen::Vector2f startAngle(90, 90);
	Eigen::Vector2f endAngle(0, 0);

	Eigen::Vector3f rightFootPosOffset(0, 0, 0);
	Eigen::Vector3f rightFootAngleOffset(0, 0, 0);
	Eigen::Vector3f leftFootPosOffset(0, 0, 0);
	Eigen::Vector3f leftFootAngleOffset(0, 0, 0);
	Eigen::Vector3f pelvisFootPosOffset(0, 0, 0);
	Eigen::Vector3f pelvisFootAngleOffset(0, 0, 0);


	Trajectory* traj = new Trajectory(rightFootPosOffset, rightFootAngleOffset, leftFootPosOffset,
			leftFootAngleOffset, pelvisFootPosOffset, pelvisFootAngleOffset, pelvisFootPosOffset, pelvisFootAngleOffset, 0, 0.04);


	Eigen::MatrixXf matrix = traj->GenerateWalk(pointA, pointD,
			endAngle, startAngle, Trajectory::ZMP, 1.0f);

	//Eigen::MatrixXf matrix = traj->GenerateKick(0.5);

/*
	Eigen::VectorXf rightInit(6);
	rightInit << 0.9119, 0.6555, 0, 0, 0, 0.78;
	Eigen::VectorXf rightFinal(6);
	rightFinal << 0.9119, 0.6555, 0, 0, 0, 0.78;

	Eigen::VectorXf leftInit(6);
	leftInit << 0.83, 0.68, 0, 0, 0, 0.8857;
	Eigen::VectorXf leftFinal(6);
	leftFinal << 0.83, 0.68, 0, 0, 0, 0.8857;

	Eigen::VectorXf pelvisInit(6);
	pelvisInit << 0.83, 0.68, 0.27, 0.886, 0, 0;
	Eigen::VectorXf pelvisFinal(6);
	pelvisFinal << 0.91, 0.665, 0.27, 0.78, 0, 0;

	Eigen::MatrixXf matrix1 = traj->GenerateMovement(rightInit, rightFinal, leftInit, leftFinal, pelvisInit, pelvisFinal, 1, 1, false);
*/

/*
	MotionControl* motion = new MotionControl(0.002f, 0.02f, 5);
	motion->Move(matrix);
*/
/*
	Eigen::VectorXf rightInit2(6);
	rightInit2 << 0.9119, 0.6555, 0, 0, 0, 0.78;
	Eigen::VectorXf rightFinal2(6);
	rightFinal2 << 0.9119, 0.6555, 0, 0, 0, 0.78;

	Eigen::VectorXf leftInit2(6);
	leftInit2 << 0.83, 0.68, 0, 0, 0, 0.8857;
	Eigen::VectorXf leftFinal2(6);
	leftFinal2 << 0.88, 0.73, 0, 0, 0, 0.665;

	Eigen::VectorXf pelvisInit2(6);
	pelvisInit2 << 0.91, 0.665, 0.27, 0.78, 0, 0;
	Eigen::VectorXf pelvisFinal2(6);
	pelvisFinal2 << 0.91, 0.665, 0.27, 0.78, 0, 0;


	Eigen::MatrixXf matrix2 = traj->GenerateMovement(rightInit2, rightFinal2, leftInit2, leftFinal2, pelvisInit2, pelvisFinal2, 1, 0, false);

	//time
	Eigen::VectorXf timeVector(2*matrix1.rows());
	timeVector = Eigen::VectorXf::LinSpaced(2*matrix1.rows(), 0, 2*matrix1.rows()*0.01f);

	Eigen::MatrixXf matrixtemp(2*matrix1.rows(), 19);
	matrixtemp << matrix1, matrix2;

	Eigen::MatrixXf matrix(2*matrix1.rows(), 20);
	matrix << timeVector, matrixtemp;
*/

	MotionControl* motion = new MotionControl(0.002f, 0.02f, 5);
	motion->Move(matrix);



	//MotionControl* motion = new MotionControl();
	//motion->Move(matrix);
/*
	std::vector<double> initialPos = motion->GetInitialQPosition();

	std::vector<double> q;
	for(int i = 0; i < matrix.rows(); ++i)
	{
		q = motion->UpdateQ((Eigen::VectorXf)matrix.row(i));
	}

*/

	ofstream myfiletraj;
	myfiletraj.open ("matrixTraj.txt");

	for(int i = 0; i < matrix.rows(); i++)
	{
		//*****************write to a file for tests************************************//
		myfiletraj << matrix.row(i) << endl;
		//*****************************************************************************//
	}

	myfiletraj.close();

	Eigen::MatrixXf matrice(3, 6);
	matrice << -0.1747, -0.1747, -0.0874, 0, 0,0,
			0, 0, -0.0319, 0, 0, 0,
			0, 0, 0, 0, 0, 0;


	Eigen::MatrixXf testPseudoInv = EigenUtils::PseudoInverse(matrice);

	return 0;
}
