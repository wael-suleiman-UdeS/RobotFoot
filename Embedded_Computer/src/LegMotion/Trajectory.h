/**
******************************************************************************^M
* @file    Trajectory.h
* @authors  Camille Hébert & Antoine Rioux
* @date    2013-11-19
* @brief   Class to calculate leg and pelvis trajectories
******************************************************************************^M
*/


#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "../../ThirdParty/Eigen/Dense"

class Trajectory
{
public:

	enum PelvisTrajectoryType {ZMP, COM};

	Trajectory();
	~Trajectory();

	Eigen::MatrixXf GenerateWalk(Eigen::Vector2f startingPoint, Eigen::Vector2f goalPoint, Eigen::Vector2f goalAngle,
			Eigen::Vector2f startingAngle, PelvisTrajectoryType pelvisTrajType, float stepTime = 3.0f, float stepHeight = 0.02f);
	Eigen::MatrixXf GenerateKick(float kickSpeedRatio, float movementTime = 1.0f);
	Eigen::MatrixXf GenerateMovement(Eigen::Vector4f& rightFootInitialPos, Eigen::Vector4f& rightFootFinalPos, Eigen::Vector4f& leftFootInitialPos,
			Eigen::Vector4f& leftFootFinalPos, Eigen::Vector4f& pelvisInitialPos, Eigen::Vector4f& pelvisFinalPos, float timeLapse);

private:

	Eigen::MatrixXf ToList(Eigen::Vector2f PointA, Eigen::Vector2f PointD, Eigen::MatrixXf LeftTrajectory, Eigen::MatrixXf RightTrajectory);

	void BezierDegre2(Eigen::VectorXf& xPositionsVector, Eigen::VectorXf& yPositionsVector, Eigen::VectorXf& angles,
							Eigen::Vector2f pointA, Eigen::Vector2f pointD, Eigen::Vector2f startAngle, Eigen::Vector2f endAngle, float dist = 0.5f);
	Eigen::Vector2f GetBezierTangent(Eigen::MatrixXf controlPoints, float t);
	float GetBezierAngle(Eigen::MatrixXf controlPoints, float t);
	void ParallelCurve(Eigen::VectorXf &xInner, Eigen::VectorXf &yInner, Eigen::VectorXf &xOuter, Eigen::VectorXf &yOuter, Eigen::VectorXf& x, Eigen::VectorXf& y);
	void GenerateSteps(Eigen::MatrixXf &rightSteps, Eigen::MatrixXf &leftSteps, Eigen::VectorXf &xInner, Eigen::VectorXf &yInner, Eigen::VectorXf &xOuter, Eigen::VectorXf &yOuter,
			Eigen::Vector2f startingPoint, Eigen::Vector2f startAngle, Eigen::VectorXf& angles);

	Eigen::MatrixXf GenerateParabollicStepsTrajectories(Eigen::MatrixXf rightSteps, Eigen::MatrixXf leftSteps, int finalMatrixSize);
	void GenerateFinalMatrixForOneStep(Eigen::MatrixXf& finalMatrix, int stepCount,
			Eigen::Vector4f& startingStepPos, Eigen::Vector4f& endingStepPos, Eigen::Vector4f& groundedFootPos,
			float singleStepTime, int startTime, int endTime, int groundedFoot);
	Eigen::MatrixXf GenerateParabollicTrajParams(Eigen::Vector4f initialPos, Eigen::Vector4f finalPos, float stepTimeLapse);
	Eigen::Vector3f GenerateParabollicTrajectory(Eigen::MatrixXf params, float currentTime);

	Eigen::MatrixXf GenerateZMP(Eigen::MatrixXf rightSteps, Eigen::MatrixXf leftSteps);
	Eigen::MatrixXf GenerateCOM(Eigen::MatrixXf zmpMatrix);

	float m_singleStepTime;
	float m_dLeg;
	float m_dStep;
	float m_stepHeight;
	float m_dTime;
	float m_ZMPHeight;
	int m_nbTrajectoryPoints;
};

#endif  //TRAJECTORY_H
