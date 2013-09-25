#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "../ThirdParty/Eigen/Dense"

class Trajectory
{
public:

Trajectory();
~Trajectory();

void BezierDegre2(Eigen::VectorXf& xPositionsVector, Eigen::VectorXf& yPositionsVector, 
	Eigen::Vector2f pointA, Eigen::Vector2f pointD, int startAngle, int endAngle, int dist = 1);

Eigen::MatrixXf ToList(Eigen::Vector2f PointA, Eigen::Vector2f PointD, Eigen::MatrixXf LeftTrajectory, Eigen::MatrixXf RightTrajectory);
Eigen::MatrixXf MXB(Eigen::Vector2f pointA, Eigen::Vector2f pointB, float increment);
Eigen::MatrixXf SpatialZMP(Eigen::Vector2f pointA, Eigen::Vector2f pointD, Eigen::MatrixXf leftTrajectory, Eigen::MatrixXf rightTrajectory, float increment);
Eigen::MatrixXf TemporalZMP(Eigen::Vector2f pointA, Eigen::Vector2f pointD, Eigen::MatrixXf leftTrajectory, Eigen::MatrixXf rightTrajectory, int Tp, float TEch);

private:

Eigen::MatrixXf AppendMatrix(Eigen::MatrixXf matrixA, Eigen::MatrixXf matrixB, int nbColumns);

};

#endif  //TRAJECTORY_H
