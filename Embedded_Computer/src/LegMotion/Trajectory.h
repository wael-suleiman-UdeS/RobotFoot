#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "../../ThirdParty/Eigen/Dense"

class Trajectory
{
public:

Trajectory();
~Trajectory();

void BezierDegre2(Eigen::VectorXf& xPositionsVector, Eigen::VectorXf& yPositionsVector, 
	Eigen::Vector2f pointA, Eigen::Vector2f pointD, int startAngle, int endAngle, int dist = 1);

Eigen::MatrixXf ToList(Eigen::Vector2f PointA, Eigen::Vector2f PointD, Eigen::MatrixXf LeftTrajectory, Eigen::MatrixXf RightTrajectory);
Eigen::MatrixXf MXB(Eigen::Vector2f pointA, Eigen::Vector2f pointB, float increment, int offset = 0);
Eigen::MatrixXf SpatialZMP(Eigen::Vector2f pointA, Eigen::Vector2f pointD, Eigen::MatrixXf leftTrajectory, Eigen::MatrixXf rightTrajectory, float increment);
Eigen::MatrixXf TemporalZMP(Eigen::MatrixXf zmpSteps, int tp, float tEch);
//Eigen::MatrixXf TemporalZMP(Eigen::Vector2f pointA, Eigen::Vector2f pointD, Eigen::MatrixXf leftTrajectory, Eigen::MatrixXf rightTrajectory, int Tp, float TEch);
void ParallelCurve(Eigen::VectorXf &xInner, Eigen::VectorXf &yInner, Eigen::VectorXf &xOuter, Eigen::VectorXf &yOuter, Eigen::VectorXf x, Eigen::VectorXf y, float d);

//////////////////////////////////////////////////////////////////////
Eigen::MatrixXf UpdateDH(float L4, float L5, Eigen::VectorXf q);
Eigen::Matrix4f MatrixHomogene(Eigen::MatrixXf DH);
Eigen::MatrixXf Jacobian(Eigen::MatrixXf DH, int returnChoice);
//////////////////////////////////////////////////////////////////////

private:

Eigen::MatrixXf AppendMatrixRow(Eigen::MatrixXf matrixA, Eigen::MatrixXf matrixB);
Eigen::MatrixXf AppendMatrixColumn(Eigen::MatrixXf matrixA, Eigen::MatrixXf matrixB);
Eigen::MatrixXf CreateCombinedMXBMatrix(Eigen::MatrixXf matrixA, Eigen::MatrixXf matrixB, float increment, int i, int j, int offset = 0);

};

#endif  //TRAJECTORY_H
