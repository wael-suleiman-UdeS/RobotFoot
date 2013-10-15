#include "ZMP.h"
#include "EigenUtils.h"

ZMP::ZMP()
{}

ZMP::~ZMP()
{}

Eigen::MatrixXf ZMP::SpatialZMP(Eigen::Vector2f pointA, Eigen::Vector2f pointD, Eigen::MatrixXf leftTrajectory, Eigen::MatrixXf rightTrajectory, float increment)
{
	//Trajectory from point A to left footprint
    Eigen::MatrixXf trajectory = EigenUtils::MXB(pointA, leftTrajectory.row(0), increment);

    for(int i =0, j = 1; i < leftTrajectory.innerSize() - 1; ++i, ++j)
    {
		//Trajectory from left to right to left foot steps
		Eigen::MatrixXf mxbMatrix = EigenUtils::CreateCombinedMXBMatrix(leftTrajectory, rightTrajectory, increment, i, j);

        Eigen::MatrixXf tempMatrix = EigenUtils::AppendMatrixRow(trajectory, mxbMatrix);
        trajectory.swap(tempMatrix);
    }

    //Append the last step (left foot) to pointD
    Eigen::MatrixXf finalStepTraj = EigenUtils::MXB(leftTrajectory.row(leftTrajectory.rows()-1), pointD, increment);
    Eigen::MatrixXf tempMatrix = EigenUtils::AppendMatrixRow(trajectory, finalStepTraj);
    trajectory.swap(tempMatrix);

    return trajectory;
}

//Mettre pointeurs
Eigen::MatrixXf ZMP::TemporalZMP(Eigen::MatrixXf zmpSteps, int tp, float tEch)
{
    float ts = 0.2*tp;

    //Create Tpn
    Eigen::VectorXf tpn = Eigen::VectorXf::LinSpaced(tp/tEch, 0, tp);

	//Initial position (X,Y,T)
    Eigen::Vector3f tempDeplacement(zmpSteps(0, 0), zmpSteps(0, 1), 0);

	Eigen::MatrixXf timeMVTMatrix;

	int attendVectorLength = tp/tEch;

    for(int i = 0; i < zmpSteps.innerSize() - 1; ++i)
    {
		Eigen::Vector2f firstStep = zmpSteps.row(i);
		Eigen::Vector2f secondStep = zmpSteps.row(i + 1);

		//Calculate trajectory between 2 footsteps
		Eigen::MatrixXf deplacementZMP = EigenUtils::MXB(firstStep, secondStep, tEch/ts);

		Eigen::MatrixXf groundedFootZMP(attendVectorLength, 2);
		groundedFootZMP << Eigen::VectorXf::Constant(attendVectorLength, firstStep(0)), Eigen::VectorXf::Constant(attendVectorLength, firstStep(1));

		Eigen::MatrixXf deplacement = EigenUtils::AppendMatrixRow(deplacementZMP, groundedFootZMP);

		Eigen::MatrixXf tempMatrix;
		if(timeMVTMatrix.innerSize() >0)
			tempMatrix = EigenUtils::AppendMatrixRow(timeMVTMatrix, deplacement);
		else
			tempMatrix = deplacement;

		timeMVTMatrix.swap(tempMatrix);
	}

	//total time
	float totalTime = (zmpSteps.innerSize() - 1)*(ts + tp);

	//Create Ttn
    Eigen::VectorXf ttn = Eigen::VectorXf::LinSpaced(totalTime/tEch, 0, totalTime);

	Eigen::MatrixXf deplacementTemporel = EigenUtils::AppendMatrixColumn(timeMVTMatrix, ttn);

	return deplacementTemporel;
}
