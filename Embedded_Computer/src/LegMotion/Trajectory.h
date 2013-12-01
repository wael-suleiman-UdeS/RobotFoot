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

	Trajectory(Eigen::Vector3f rightFootPosOffset, Eigen::Vector3f rightFootAngleOffset,
			Eigen::Vector3f leftFootPosOffset, Eigen::Vector3f leftFootAngleOffset,
			Eigen::Vector3f pelvisPosOffset, Eigen::Vector3f pelvisAngleOffset);
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
			Eigen::VectorXf& startingStepPos, Eigen::VectorXf& endingStepPos, Eigen::VectorXf& groundedFootPos,
			float singleStepTime, int startTime, int endTime, int groundedFoot);
	Eigen::MatrixXf GenerateParabollicTrajParams(Eigen::VectorXf initialPos, Eigen::VectorXf finalPos, float stepTimeLapse);
	Eigen::VectorXf GenerateParabollicTrajectory(Eigen::MatrixXf params, float currentTime);

	Eigen::MatrixXf GenerateZMP(Eigen::MatrixXf rightSteps, Eigen::MatrixXf leftSteps, int finalMatrixSize);
	void GenerateZMPStepTransfer(Eigen::MatrixXf& trajectoryMatrix, Eigen::VectorXf startingPos, Eigen::VectorXf endingPost, int stepIndex);
	Eigen::MatrixXf GenerateCOM(Eigen::MatrixXf zmpMatrix);

	Eigen::Vector3f m_vRightFootPosOffset;
	Eigen::Vector3f m_vRightFootAngleOffset;
	Eigen::Vector3f m_vLeftFootPosOffset;
	Eigen::Vector3f m_vLeftFootAngleOffset;
	Eigen::Vector3f m_vPelvisPosOffset;
	Eigen::Vector3f m_vPelvisAngleOffset;

	float m_singleStepTime;
	float m_dLeg;
	float m_dStep;
	float m_stepHeight;
	float m_dTime;
	float m_ZMPHeight;
	int m_nbTrajectoryPoints;
};

#endif  //TRAJECTORY_H
