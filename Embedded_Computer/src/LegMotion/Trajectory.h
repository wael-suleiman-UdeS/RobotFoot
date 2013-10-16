#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "../../ThirdParty/Eigen/Dense"

class Trajectory
{
public:

Trajectory();
~Trajectory();

void GenerateWalk(Eigen::Vector2f startingPoint, Eigen::Vector2f goalPoint, float goalAngle = 0, float startingAngle = 0);
void GenerateKick();

private:

Eigen::MatrixXf ToList(Eigen::Vector2f PointA, Eigen::Vector2f PointD, Eigen::MatrixXf LeftTrajectory, Eigen::MatrixXf RightTrajectory);
void BezierDegre2(Eigen::VectorXf& xPositionsVector, Eigen::VectorXf& yPositionsVector, 
						Eigen::Vector2f pointA, Eigen::Vector2f pointD, int startAngle, int endAngle, int dist = 1);
void ParallelCurve(Eigen::VectorXf &xInner, Eigen::VectorXf &yInner, Eigen::VectorXf &xOuter, Eigen::VectorXf &yOuter, Eigen::VectorXf& x, Eigen::VectorXf& y);
void GenerateSteps(Eigen::MatrixXf &rightSteps, Eigen::MatrixXf &leftSteps, Eigen::VectorXf &xInner, Eigen::VectorXf &yInner, Eigen::VectorXf &xOuter, Eigen::VectorXf &yOuter,
		Eigen::Vector2f startingPoint, int startAngle);

float m_samplingTime;
float m_singleStepTime;
float m_dLeg;
float m_dStep;
int m_nbTrajectoryPoints;
};

#endif  //TRAJECTORY_H
