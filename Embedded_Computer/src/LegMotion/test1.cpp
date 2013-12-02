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
	Eigen::Vector2f pointD(1, 0);
	//Eigen::Vector2f pointD(1, 1.3);
	Eigen::Vector2f startAngle(0, 0);
	//Eigen::Vector2f startAngle(90, 90);
	Eigen::Vector2f endAngle(0, 0);

	Eigen::Vector3f rightFootPosOffset(0, 0, 0);
	Eigen::Vector3f rightFootAngleOffset(0, 0, 0);
	Eigen::Vector3f leftFootPosOffset(0, 0, 0);
	Eigen::Vector3f leftFootAngleOffset(0, 0, 0);
	Eigen::Vector3f pelvisFootPosOffset(0, 0, 0);
	Eigen::Vector3f pelvisFootAngleOffset(-0.03, -0.17, 0);


	Trajectory* traj = new Trajectory(rightFootPosOffset, rightFootAngleOffset, leftFootPosOffset,
			leftFootAngleOffset, pelvisFootPosOffset, pelvisFootAngleOffset, pelvisFootPosOffset, pelvisFootAngleOffset, 0.03, 0.04);
	Eigen::MatrixXf matrix = traj->GenerateWalk(pointA, pointD,
			endAngle, startAngle, Trajectory::COM, 1.0f);

	//Eigen::MatrixXf matrix = traj->GenerateKick(0.5);

	MotionControl* motion = new MotionControl(0.002f, 0.02f, 5);
	motion->Move(matrix);

	//Eigen::MatrixXf matrix = traj->GenerateKick(0.5);
/*
	Eigen::Vector4f rightInit(0.037, 0.0436, 0, 0);
	Eigen::Vector4f rightFinal(0.037, 0.0436, 0, 0);

	Eigen::Vector4f leftInit(-0.037, 0.0, 0, 0);
	Eigen::Vector4f leftFinal(-0.037, 0.0848, 0, 0);

	Eigen::Vector4f pelvisInit(0.037, 0.0432, 0.3, 0);
	Eigen::Vector4f pelvisFinal(0.037, 0.0432, 0.3, 0);

	Eigen::MatrixXf matrix = traj->GenerateMovement(rightInit, rightFinal, leftInit, leftFinal, pelvisInit, pelvisFinal, 1);
*/
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
