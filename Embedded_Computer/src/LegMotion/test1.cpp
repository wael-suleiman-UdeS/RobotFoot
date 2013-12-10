#include "Trajectory.h"
#include "MotionControl.h"
#include "EigenUtils.h"
#include "DenavitHartenberg.h"
#include "TestCourbe.h"

#include "../../ThirdParty/Eigen/Dense"

#include <iostream>
#include <fstream>
#include <cstdlib>
using namespace std;

int main(int argc, char* argv[]) {

/*
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

	Eigen::MatrixXf matrix = traj->GenerateWalk(pointA, pointD, endAngle, startAngle, Trajectory::PelvisTrajectoryType::ZMP, 1.0f, 0.04);

*/

	string rightTraj = "config/traj_CourbeRight.txt";
	string leftTraj = "config/traj_CourbeLeft.txt";
	string pelvisTraj = "config/traj_CourbePelvis.txt";
	string fixedFootTraj = "config/traj_FixedFoot.txt";

	Eigen::MatrixXf rightTrajMatrix = TestCourbe::GetTrajectoriesFromFile(rightTraj);
	Eigen::MatrixXf leftTrajMatrix = TestCourbe::GetTrajectoriesFromFile(leftTraj);
	Eigen::MatrixXf pelvisTrajMatrix = TestCourbe::GetTrajectoriesFromFile(pelvisTraj);
	Eigen::VectorXf fixedFootVector = TestCourbe::GetFixedFootFromFile(fixedFootTraj);

	int matrixSize = (pelvisTrajMatrix.rows()-1)/0.01f;
	Eigen::VectorXf time = Eigen::VectorXf::LinSpaced(matrixSize, 0, matrixSize*0.01);

	Eigen::Vector3f rightFootPosOffset(0, 0, 0);
	Eigen::Vector3f rightFootAngleOffset(0, 0, 0);
	Eigen::Vector3f leftFootPosOffset(0, 0, 0);
	Eigen::Vector3f leftFootAngleOffset(0, 0, 0);
	Eigen::Vector3f pelvisFootPosOffset(0, 0, 0);
	Eigen::Vector3f pelvisFootAngleOffset(0, 0, 0);


	Trajectory* traj = new Trajectory(rightFootPosOffset, rightFootAngleOffset, leftFootPosOffset,
			leftFootAngleOffset, pelvisFootPosOffset, pelvisFootAngleOffset, pelvisFootPosOffset, pelvisFootAngleOffset, 0, 0.04);

	Eigen::MatrixXf matrixWOTime(0, 0);
	int size = 1/0.01;
	Eigen::MatrixXf matrixMVT(size, 20);

	Eigen::VectorXf rightStart(6);
	Eigen::VectorXf rightFinal(6);
	Eigen::VectorXf leftStart(6);
	Eigen::VectorXf leftFinal(6);
	Eigen::VectorXf pelvisStart(6);
	Eigen::VectorXf pelvisFinal(6);


	for(int i = 0; i < pelvisTrajMatrix.rows(); i++)
	{
		if(pelvisTrajMatrix.rows() > i+1)
		{
			Eigen::MatrixXf tempMatrix(matrixWOTime.rows()+size, 19);

			rightStart = rightTrajMatrix.row(i);
			rightFinal = rightTrajMatrix.row(i+1);
			leftStart = leftTrajMatrix.row(i);
			leftFinal = leftTrajMatrix.row(i+1);
			pelvisStart = pelvisTrajMatrix.row(i);
			pelvisFinal = pelvisTrajMatrix.row(i+1);

			matrixMVT = traj->GenerateMovement(rightStart, rightFinal, leftStart, leftFinal, pelvisStart,
					pelvisFinal, 1.0f, (int)fixedFootVector(i));

			if(matrixWOTime.rows() > 0)
			{
				tempMatrix << matrixWOTime, matrixMVT;
				matrixWOTime.swap(tempMatrix);
			}
			else
				matrixWOTime.swap(matrixMVT);

		}
	}

	Eigen::MatrixXf matrix(matrixSize, 20);
	matrix << time, matrixWOTime;

	MotionControl* motion = new MotionControl(0.002f, 0.02f, 5);
	motion->Move(matrix);




	ofstream myfiletraj;
	myfiletraj.open ("matrixTraj.txt");

	for(int i = 0; i < matrix.rows(); i++)
	{
		myfiletraj << matrix.row(i) << endl;
	}

	myfiletraj.close();

	Eigen::MatrixXf matrice(3, 6);
	matrice << -0.1747, -0.1747, -0.0874, 0, 0,0,
			0, 0, -0.0319, 0, 0, 0,
			0, 0, 0, 0, 0, 0;




	return 0;
}
