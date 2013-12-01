/**
******************************************************************************^M
* @file    Trajectory.cpp
* @authors  Camille Hébert & Antoine Rioux
* @date    2013-11-19
* @brief   Class to calculate leg and pelvis trajectories
******************************************************************************^M
*/


#include "Trajectory.h"

#define Debug

#include "EigenUtils.h"
#include <math.h>
#include <fstream>
#include <iostream>
#include "../../ThirdParty/Eigen/Dense"
#include <stdlib.h>

using namespace std;


/** \brief Constructor
 *
 *
 */
Trajectory::Trajectory(Eigen::Vector3f rightFootPosOffset, Eigen::Vector3f rightFootAngleOffset,
		Eigen::Vector3f leftFootPosOffset, Eigen::Vector3f leftFootAngleOffset, Eigen::Vector3f rightPelvisPosOffset,
		Eigen::Vector3f rightPelvisAngleOffset, Eigen::Vector3f leftPelvisPosOffset, Eigen::Vector3f leftPelvisAngleOffset)
: m_vRightFootPosOffset(rightFootPosOffset)
, m_vRightFootAngleOffset(rightFootAngleOffset)
, m_vLeftFootPosOffset(leftFootPosOffset)
, m_vLeftFootAngleOffset(leftFootAngleOffset)
, m_vRightPelvisPosOffset(rightPelvisPosOffset)
, m_vRightPelvisAngleOffset(rightPelvisAngleOffset)
, m_vLeftPelvisPosOffset(leftPelvisPosOffset)
, m_vLeftPelvisAngleOffset(leftPelvisAngleOffset)
, m_dLeg(0.037f)
, m_dStep(0.03f)
, m_dTime(0.01f)
, m_nbTrajectoryPoints(101)
{
	string line;
	ifstream zmpFile("config/zmp.txt");
	if (zmpFile.is_open())
	{
		getline (zmpFile,line);
		m_ZMPHeight = float(atof(line.c_str()));
		zmpFile.close();
	}
	else
		m_ZMPHeight = 0.3f;
}

/** \brief Destructor
 *
 *
 */
Trajectory::~Trajectory()
{}

/** \brief Generate a matrix that contains every information necessary to generate a walk
 *
 * \param startingPoint Eigen::VectorX2f: Starting position in x and y
 * \param goalPoint Eigen::Vector2f: Goal position in x and y
 * \param goalAngle Eigen::Vector2f: Starting orientation of the robot in x and y
 * \param startingAngle Eigen::Vector2f: Goal orientation of the robot in x and y
 * \param stepTime float: Time required to perform a single step (optional)
 * \param stepHeight float: Height at which the foot is raised when performing a step (optional)
 *
 */
Eigen::MatrixXf Trajectory::GenerateWalk(Eigen::Vector2f startingPoint, Eigen::Vector2f goalPoint, Eigen::Vector2f goalAngle,
		Eigen::Vector2f startingAngle, PelvisTrajectoryType pelvisTrajType, float stepTime, float stepHeight)
{
	m_singleStepTime = stepTime;
	m_stepHeight = stepHeight;

	Eigen::VectorXf xTrajectory(m_nbTrajectoryPoints);
	Eigen::VectorXf yTrajectory(m_nbTrajectoryPoints);
	Eigen::VectorXf angles(m_nbTrajectoryPoints);

	//2nd degree Bezier curve
	BezierDegre2(xTrajectory, yTrajectory, angles, startingPoint, goalPoint, startingAngle, goalAngle);

	Eigen::VectorXf xInner(xTrajectory.innerSize());
	Eigen::VectorXf yInner(yTrajectory.innerSize());
	Eigen::VectorXf xOuter(xTrajectory.innerSize());
	Eigen::VectorXf yOuter(yTrajectory.innerSize());

	//Parallel curve
	ParallelCurve(xInner, yInner, xOuter, yOuter, xTrajectory, yTrajectory);

	Eigen::MatrixXf rightSteps;
	Eigen::MatrixXf leftSteps;

	//Generate steps on the ground to follow the parallel curve
	GenerateSteps(rightSteps, leftSteps, xInner, yInner, xOuter, yOuter, startingPoint, startingAngle, angles);

	//Calculate ZMP
	int finalMatrixSize = (m_singleStepTime/m_dTime)*(rightSteps.rows() + leftSteps.rows() - 2)*2 + (m_singleStepTime/m_dTime);
	Eigen::MatrixXf zmp = GenerateZMP(rightSteps, leftSteps, finalMatrixSize);

	Eigen::MatrixXf pelvisTraj;
	//If we're using the Center of Mass (Riccati technique) instead of the zmp
	if(pelvisTrajType == COM)
	{
		Eigen::MatrixXf com = GenerateCOM(zmp);

		#ifdef Debug
		ofstream myfiletraj;
		myfiletraj.open ("com.txt");

		for(int i = 0; i < com.rows(); i++)
		{
			myfiletraj << com.row(i) << endl;
		}

		myfiletraj.close();
		#endif

		pelvisTraj = com;
	}
	else
		pelvisTraj = zmp;

	//Create trajectory for moving foot
	Eigen::MatrixXf trajectoryMatrix = GenerateParabollicStepsTrajectories(rightSteps, leftSteps, finalMatrixSize);

	//Create a time vector
	Eigen::VectorXf time = Eigen::VectorXf::LinSpaced(finalMatrixSize, 0, finalMatrixSize*m_dTime);

	//Append ZMP to final matrix
	Eigen::MatrixXf finalMatrix(finalMatrixSize, 20);
	finalMatrix << time, trajectoryMatrix, pelvisTraj;

#ifdef Debug
ofstream myfiletraj;
myfiletraj.open ("matrixTraj.txt");

for(int i = 0; i < finalMatrix.rows(); i++)
{
	myfiletraj << finalMatrix.row(i) << endl;
}

myfiletraj.close();
#endif

	return finalMatrix;
}

/** \brief Generates a matrix which contains the necessary information to perform a kick
 *
 */
Eigen::MatrixXf Trajectory::GenerateKick( float kickSpeedRatio, float movementTime )
{
	//Clamp 0.1 to 1
	kickSpeedRatio = min(max(kickSpeedRatio, 0.1f), 1.0f);

	Eigen::Vector4f startingPointR(0.037f, 0.0f, 0.0f, 0.0f);
	Eigen::Vector4f startingPointL(-0.037f, 0.0f, 0.0f, 0.0f);
	Eigen::Vector4f startingPointP(0.0f, 0.0f, m_ZMPHeight, 0.0f);
	Eigen::Vector4f zmpOverFixedFootP(0.05f, 0.0f, m_ZMPHeight, 0.0f);
	Eigen::Vector4f leftFootRaisedL(-0.037f, 0.0f, 0.04f, 0.0f);
	Eigen::Vector4f kickingFootBackL(-0.037f, -0.08f, 0.04f, 0.0f);
	Eigen::Vector4f kickingFootForwardL(-0.037f, 0.1f, 0.04f, 0.0f);

	int matrixSize = 5*movementTime/m_dTime + kickSpeedRatio*movementTime/m_dTime;
	Eigen::MatrixXf finalMatrix(matrixSize, 13);

	Eigen::MatrixXf zmpOverFootMatrix;
	Eigen::MatrixXf raisedFootMatrix;
	Eigen::MatrixXf footBackMatrix;
	Eigen::MatrixXf footFrontMatrix;
	Eigen::MatrixXf footBackToNormalMatrix;
	Eigen::MatrixXf zmpBackToNormalMatrix;


	//Bring zmp over fixed foot
	zmpOverFootMatrix = GenerateMovement(startingPointR, startingPointR, startingPointL, startingPointL, startingPointP, zmpOverFixedFootP, movementTime);
	//Raise foot
	raisedFootMatrix = GenerateMovement(startingPointR, startingPointR, startingPointL, leftFootRaisedL, zmpOverFixedFootP, zmpOverFixedFootP, movementTime);
	//Bring foot back
	footBackMatrix = GenerateMovement(startingPointR, startingPointR, leftFootRaisedL, kickingFootBackL, zmpOverFixedFootP, zmpOverFixedFootP, movementTime);
	//Bring foot front to kick
	footFrontMatrix = GenerateMovement(startingPointR, startingPointR, kickingFootBackL, kickingFootForwardL, zmpOverFixedFootP, zmpOverFixedFootP, movementTime*kickSpeedRatio);
	//Bring foot back in normal position
	footBackToNormalMatrix = GenerateMovement(startingPointR, startingPointR, kickingFootForwardL, startingPointL, zmpOverFixedFootP, zmpOverFixedFootP, movementTime);
	//Bring zmp back to normal position
	zmpBackToNormalMatrix = GenerateMovement(startingPointR, startingPointR, startingPointL, startingPointL, zmpOverFixedFootP, startingPointP, movementTime);

	finalMatrix << zmpOverFootMatrix, raisedFootMatrix, footBackMatrix, footFrontMatrix, footBackToNormalMatrix, zmpBackToNormalMatrix;

	return finalMatrix;
}

Eigen::MatrixXf Trajectory::GenerateMovement(Eigen::Vector4f& rightFootInitialPos, Eigen::Vector4f& rightFootFinalPos, Eigen::Vector4f& leftFootInitialPos,
		Eigen::Vector4f& leftFootFinalPos, Eigen::Vector4f& pelvisInitialPos, Eigen::Vector4f& pelvisFinalPos, float timeLapse)
{
	//final matrix
	int finalMatrixSize = timeLapse/m_dTime;
	Eigen::MatrixXf finalMatrix(finalMatrixSize, 13);

	Eigen::MatrixXf paramsRightFoot(4,3);
	Eigen::MatrixXf paramsLeftFoot(4,3);
	Eigen::MatrixXf paramsPelvis(4,3);

	paramsRightFoot = GenerateParabollicTrajParams(rightFootInitialPos, rightFootFinalPos, timeLapse);
	paramsLeftFoot = GenerateParabollicTrajParams(leftFootInitialPos, leftFootFinalPos, timeLapse);
	paramsPelvis = GenerateParabollicTrajParams(pelvisInitialPos, pelvisFinalPos, timeLapse);

	Eigen::Vector3f currentRightFootPos;
	Eigen::Vector3f currentLeftFootPos;
	Eigen::Vector3f currentPelvisPos;

	//Determine grounded foot
	//If the right foot is moving, set the left foot as grounded (even if the left foot is also moving for protection)
	bool groundedFoot = (rightFootFinalPos - rightFootInitialPos).norm() > 0.01;

	for(int time = 0; time < finalMatrixSize; time ++)
	{
		currentPelvisPos = GenerateParabollicTrajectory(paramsPelvis, m_dTime*time);

		if(groundedFoot == 1)	//Left foot is 1, right foot is moving
		{
			currentRightFootPos = GenerateParabollicTrajectory(paramsRightFoot, m_dTime*time);

			//Right foot moving
			finalMatrix(time, 1) = currentRightFootPos(0);	//x
			finalMatrix(time, 2) = currentRightFootPos(1);	//y
			finalMatrix(time, 3) = currentRightFootPos(2);	//z
			finalMatrix(time, 4) = rightFootInitialPos(3) + ((rightFootFinalPos(3) - rightFootInitialPos(3))/timeLapse)*time;	//angle
			//Left foot position on the ground
			finalMatrix(time, 5) = leftFootInitialPos(0);	//x
			finalMatrix(time, 6) = leftFootInitialPos(1);	//y
			finalMatrix(time, 7) = leftFootInitialPos(2);	//z
			finalMatrix(time, 8) = leftFootInitialPos(3);	//angle
		}
		else	//Right foot is 0, left foot is moving
		{
			currentLeftFootPos = GenerateParabollicTrajectory(paramsLeftFoot, m_dTime*time);

			//Right foot position on the ground
			finalMatrix(time, 1) = rightFootInitialPos(0);	//x
			finalMatrix(time, 2) = rightFootInitialPos(1);	//y
			finalMatrix(time, 3) = rightFootInitialPos(2);	//z
			finalMatrix(time, 4) = rightFootInitialPos(3);	//angle
			//Left foot moving
			finalMatrix(time, 5) = currentLeftFootPos(0);	//x
			finalMatrix(time, 6) = currentLeftFootPos(1);	//y
			finalMatrix(time, 7) = currentLeftFootPos(2);	//z
			finalMatrix(time, 8) = leftFootInitialPos(3) + ((leftFootFinalPos(3) - leftFootInitialPos(3))/timeLapse)*time;	//angle
		}

		//time
		finalMatrix(time, 0) = time*m_dTime;
		//grounded foot
		finalMatrix(time, 9) = groundedFoot;	// 0 = right, 1 = left foot
		//pelvis
		finalMatrix(time, 10) = currentPelvisPos(0);	//x
		finalMatrix(time, 11) = currentPelvisPos(1);	//y
		finalMatrix(time, 12) = currentPelvisPos(2);	//z
	}

	return finalMatrix;
}

/** \brief Generate a trajectory for a 2nd degree Bezier curve
 *
 * \param xTrajectory Eigen::VectorXf&: Points in X for the trajectory
 * \param yTrajectory Eigen::VectorXf&: Points in Y for the trajectory
 * \param angles Eigen::VectorXf&: Every angle following the bezier curve
 * \param pointA Eigen::Vector2f: Starting point
 * \param pointD Eigen::Vector2f: Ending point
 * \param startAngle Eigen::Vector2f: Starting angle (x, y)
 * \param endAngle Eigen::Vector2f: Ending angle (x, y)
 * \param dist int: optional parameter, distance
 *
 */
void Trajectory::BezierDegre2(Eigen::VectorXf& xTrajectory, Eigen::VectorXf& yTrajectory, Eigen::VectorXf& angles,
	Eigen::Vector2f pointA, Eigen::Vector2f pointD, Eigen::Vector2f startAngle, Eigen::Vector2f endAngle, float dist)
{
	//Problem solve?
	dist = (pointD - pointA).norm()/2;

	Eigen::Vector2f pointB(
		pointA(0) + dist*cos(startAngle(0)*M_PI/180.0),
		pointA(1) + dist*sin(startAngle(1)*M_PI/180.0));

	Eigen::Vector2f pointC(
		pointD(0) - dist*cos(endAngle(0)*M_PI/180.0),
		pointD(1) - dist*sin(endAngle(1)*M_PI/180.0));

	Eigen::MatrixXf controlPoints(4,2);
	controlPoints.row(0) = pointA;
	controlPoints.row(1) = pointB;
	controlPoints.row(2) = pointC;
	controlPoints.row(3) = pointD;

	//t varie entre 0 et 1. echantillonage (plus "t" est petit, plus la courbe est lisse)
	for(int i = 0; i < m_nbTrajectoryPoints; ++i)
	{
		float t = i*m_dTime;
		xTrajectory(i) = pointA(0)*pow(1-t,3) + 3*pointB(0)*t*pow(1-t,2) + 3*pointC(0)*pow(t,2)*(1-t) + pointD(0)*pow(t,3);
		yTrajectory(i) = pointA(1)*pow(1-t,3) + 3*pointB(1)*t*pow(1-t,2) + 3*pointC(1)*pow(t,2)*(1-t) + pointD(1)*pow(t,3);
		angles(i) = GetBezierAngle(controlPoints,t);
	}

}

/** \brief Finds the tangent of a particular point on the bezier curve
 *
 * \param controlPoints Eigen::MatrixXf: The control points used to create the bezier curve
 * \param t float: Sampling time
 *
 */
Eigen::Vector2f Trajectory::GetBezierTangent(Eigen::MatrixXf controlPoints, float t)
{
	if(controlPoints.rows() < 2)
		return controlPoints.row(0);
	else
	{
		Eigen::Vector2f q0 = GetBezierTangent(controlPoints.topRows(controlPoints.rows()-1), t);
		Eigen::Vector2f q1 = GetBezierTangent(controlPoints.bottomRows(controlPoints.rows()-1), t);
		return (1-t)*q0 + (t*q1);
	}
}

/** \brief Finds the angles along the bezier curve
 *
 * \param controlPoints Eigen::MatrixXf: The control points used to create the bezier curve
 * \param t float: Sampling time
 *
 */
float Trajectory::GetBezierAngle(Eigen::MatrixXf controlPoints, float t)
{
	Eigen::Vector2f q0 = GetBezierTangent(controlPoints.topRows(controlPoints.rows()-1), t);
	Eigen::Vector2f q1 = GetBezierTangent(controlPoints.bottomRows(controlPoints.rows()-1), t);
	return atan2(q1(1) - q0(1), q1(0) - q0(0))*180/M_PI;
}

/** \brief Generates 2 curves for each foot parallel to the bezier curve
 *
 * \param xInner Eigen::VectorXf&: Points in X for the left foot trajectory
 * \param yInner Eigen::VectorXf&: Points in Y for the left foot trajectory
 * \param xOuter Eigen::VectorXf&: Points in X for the right foot trajectory
 * \param yOuter Eigen::VectorXf&: Points in Y for the right foot trajectory
 * \param x Eigen::VectorXf&: Bezier curve's points in X
 * \param y Eigen::VectorXf&: Bezier curve's points in Y
 *
 */
void Trajectory::ParallelCurve(Eigen::VectorXf &xInner, Eigen::VectorXf &yInner, Eigen::VectorXf &xOuter, Eigen::VectorXf &yOuter, Eigen::VectorXf& x, Eigen::VectorXf& y)
{
	//remove 1 element, but shoulnt (matlab gradient does not)
	Eigen::VectorXf dx(x.rows());
	dx << x.row(1)-x.row(0), x.bottomRows(x.rows()-1) - x.topRows(x.rows()-1);
	Eigen::VectorXf dy(y.rows());
	dy << y.row(1)-y.row(0), y.bottomRows(y.rows()-1) - y.topRows(y.rows()-1);

	Eigen::VectorXf dx2(dx.rows());
	dx2 << dx.row(1)-dx.row(0), dx.bottomRows(dx.rows()-1) - dx.topRows(dx.rows()-1);
	Eigen::VectorXf dy2(dy.rows());
	dy2 << dy.row(1)-dy.row(0), dy.bottomRows(dy.rows()-1) - dy.topRows(dy.rows()-1);

	Eigen::MatrixXf unv(dy.rows(), dy.cols()+ dx.cols());
	unv << dy, -dx;

	float norm = 0;
	for(int i = 0; i < dy.rows(); i++)
	{
		norm = unv.row(i).norm();
		unv.row(i) /= norm;
	}

	for(int i = 0; i < x.innerSize(); ++i)
	{
		yInner(i) = x(i) + unv(i, 0)*m_dLeg;
		xInner(i) = y(i) + unv(i, 1)*m_dLeg;

		yOuter(i) = x(i) - unv(i, 0)*m_dLeg;
		xOuter(i) = y(i) - unv(i, 1)*m_dLeg;
	}
}

/** \brief Generates 2d (x,y) steps on the ground to follow the bezier curve
 *
 * \param rightSteps Eigen::MatrixXf&: A list of every right foot's steps (x,y)
 * \param leftSteps Eigen::MatrixXf&: A list of every left foot's steps (x,y)
 * \param xInner Eigen::VectorXf&: Points in X for the left foot trajectory
 * \param yInner Eigen::VectorXf&: Points in Y for the left foot trajectory
 * \param xOuter Eigen::VectorXf&: Points in X for the right foot trajectory
 * \param yOuter Eigen::VectorXf&: Points in Y for the right foot trajectory
 * \param startingPoint Eigen::Vector2f&: The robot's starting position
 * \param startAngle Eigen::Vector2f&: The starting orientation
 * \param angles Eigen::VectorXf&: Bezier curve's angles
 *
 */
void Trajectory::GenerateSteps(Eigen::MatrixXf &rightSteps, Eigen::MatrixXf &leftSteps, Eigen::VectorXf &xInner, Eigen::VectorXf &yInner, Eigen::VectorXf &xOuter, Eigen::VectorXf &yOuter,
		Eigen::Vector2f startingPoint, Eigen::Vector2f startAngle, Eigen::VectorXf& angles)
{
	Eigen::Vector3f currentLeftStepPos;
	currentLeftStepPos(0) = startingPoint(1) - (m_dLeg*cos(startAngle(1)*M_PI/180.0));
	currentLeftStepPos(1) = startingPoint(0) + (m_dLeg*sin(startAngle(0)*M_PI/180.0));
	currentLeftStepPos(2) = angles(0);

	Eigen::Vector3f currentRightStepPos;
	currentRightStepPos(0) = startingPoint(1) + (m_dLeg*cos(startAngle(1)*M_PI/180.0));
	currentRightStepPos(1) = startingPoint(0) - (m_dLeg*sin(startAngle(0)*M_PI/180.0));
	currentRightStepPos(2) = angles(0);

	Eigen::MatrixXf trajL(xInner.rows() + 1, 3);
	Eigen::MatrixXf trajR(xOuter.rows() + 1, 3);

	//Initialize firstStep
	trajL.row(0) = currentLeftStepPos;
	trajR.row(0) = currentRightStepPos;

	float dStepSquared = m_dStep*m_dStep;
	float distLeft;
	float distRight;

	int trajIndex = 1;
	for(int i = 0; i < xInner.rows(); i++)
	{
		distLeft = pow(xInner(i) - currentLeftStepPos(0), 2) + pow(yInner(i) - currentLeftStepPos(1), 2);
		distRight = pow(xOuter(i) - currentRightStepPos(0), 2) + pow(yOuter(i) - currentRightStepPos(1), 2);

		if(distLeft >= dStepSquared || distRight >= dStepSquared)
		{
			currentLeftStepPos(0) = xInner(i);
			currentLeftStepPos(1) = yInner(i);
			currentLeftStepPos(2) = angles(i);

			currentRightStepPos(0) = xOuter(i);
			currentRightStepPos(1) = yOuter(i);
			currentRightStepPos(2) = angles(i);

			trajL.row(trajIndex) = currentLeftStepPos;
			trajR.row(trajIndex) = currentRightStepPos;
			trajIndex++;
		}
	}

	int xOuterLength = xOuter.rows() - 1;
	int yOuterLength = yOuter.rows() - 1;
	if(xOuter(xOuterLength) != trajR(trajIndex - 1, 0) && yOuter(yOuterLength) != trajR(trajIndex - 1, 1))
	{
		trajR(trajIndex, 0) = xOuter(xOuterLength);
		trajR(trajIndex, 1) = yOuter(yOuterLength);

		//Set the right size for trajR
		trajR.conservativeResize(trajIndex + 1, Eigen::NoChange);
	}
	else
	{
		//Set the right size for trajR
		trajR.conservativeResize(trajIndex, Eigen::NoChange);
	}

	int xInnerLength = xInner.rows() - 1;
	int yInnerLength = yInner.rows() - 1;
	if(xInner(xInnerLength) != trajL(trajIndex - 1, 0) && yInner(yInnerLength) != trajL(trajIndex - 1, 1))
	{
		trajL(trajIndex, 0) = xInner(xInnerLength);
		trajL(trajIndex, 1) = yInner(yInnerLength);

		//Set the right size for trajL
		trajL.conservativeResize(trajIndex + 1, Eigen::NoChange);
	}
	else
	{
		//Set the right size for trajL
		trajL.conservativeResize(trajIndex, Eigen::NoChange);
	}

	//Create arrays for right and left steps
	int trajLSize = trajL.rows();

	rightSteps.resize(trajLSize/2 + 2, 3);
	leftSteps.resize(trajLSize/2 + 1, 3);

	//Add the first steps
	rightSteps.row(0) = trajR.row(0);
	leftSteps.row(0) = trajL.row(0);

	//Alternate between right and left step
	int stepIndex = 1;
	for(int i = 1; i < trajLSize; i++)
	{
		if(i % 2)
			rightSteps.row(stepIndex) = trajR.row(i);
		else
		{
			leftSteps.row(stepIndex) = trajL.row(i);
			stepIndex++;
		}
	}

	//Add the last steps
	rightSteps.row(trajLSize/2 + 1) = trajR.row(trajLSize - 1);
	leftSteps.row(trajLSize/2) = trajL.row(trajLSize - 1);
}

/** \brief Generates the trajectories to move a foot from step to step,
 *  Also Generates the final matrix with time, feet positions and which foot stands on the ground
 *
 * \param rightSteps Eigen::MatrixXf&: A list of every right foot's steps (x, y, angle)
 * \param leftSteps Eigen::MatrixXf&: A list of every left foot's steps (x, y, angle)
 * \param finalMatrix Eigen::MatrixXf&: The final matrix :
 * rightStepX, rightStepY, rightStepZ, rightStepAngle, leftStepX, leftStepY, leftStepZ, leftStepAngle, GroundedFoot(0 = right, 1 = left)
 *
 */
Eigen::MatrixXf Trajectory::GenerateParabollicStepsTrajectories(Eigen::MatrixXf rightSteps, Eigen::MatrixXf leftSteps, int finalMatrixSize)
{
	//Final matrix:
	//rightStepX, rightStepY, rightStepZ, rightStepAngle, leftStepX, leftStepY, leftStepZ, leftStepAngle, GroundedFoot(0 = right, 1 = left)
	Eigen::MatrixXf finalMatrix(finalMatrixSize, 13);
	finalMatrix = Eigen::MatrixXf::Zero(finalMatrixSize, 13);

	Eigen::VectorXf groundedFoot(6);
	Eigen::VectorXf startingStepPos(6);
	Eigen::VectorXf endStepPos(6);

	int nbSteppingTimeStamps (m_singleStepTime/m_dTime);

	int stepCount = 0;
	for(int i = 0; i < rightSteps.rows() - 1; i++)
	{
		//*******right foot**********//
		//Rise the right foot
		Eigen::Vector3f groundedFootAngle = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
		groundedFootAngle(2) += leftSteps(i, 2);
		groundedFoot << leftSteps(i, 0), leftSteps(i, 1), 0.0f, groundedFootAngle;

		Eigen::Vector3f startingStepFootAngle = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
		startingStepFootAngle(2) += rightSteps(i, 2);
		startingStepPos << rightSteps(i, 0), rightSteps(i, 1), 0.0f, startingStepFootAngle;

		Eigen::Vector3f endStepFootAngle = m_vRightFootAngleOffset;
		endStepFootAngle(2) += (rightSteps(i + 1, 2) - rightSteps(i, 2))/2 + rightSteps(i, 2);
		endStepPos << ((rightSteps(i + 1, 0) - rightSteps(i, 0))/2) + rightSteps(i, 0) + m_vRightFootPosOffset(0),
				((rightSteps(i + 1, 1) - rightSteps(i, 1))/2) + rightSteps(i, 1) + m_vRightFootPosOffset(1), m_stepHeight + m_vRightFootPosOffset(2), endStepFootAngle;

		GenerateFinalMatrixForOneStep(finalMatrix, stepCount, startingStepPos, endStepPos, groundedFoot,
				m_singleStepTime/2, 0, nbSteppingTimeStamps/2, 1);

		//Lower the right foot
		startingStepPos = endStepPos;
		endStepFootAngle = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
		endStepFootAngle(2) += rightSteps(i + 1, 2);
		endStepPos << rightSteps(i + 1, 0), rightSteps(i + 1, 1), 0.0f, endStepFootAngle;
		GenerateFinalMatrixForOneStep(finalMatrix, stepCount, startingStepPos, endStepPos, groundedFoot,
				m_singleStepTime/2, nbSteppingTimeStamps/2, nbSteppingTimeStamps, 1);

		stepCount++;

		int timeMultiplier = nbSteppingTimeStamps*stepCount;
		Eigen::MatrixXf noMovement(nbSteppingTimeStamps, finalMatrix.cols());
		for(int i = 0; i < nbSteppingTimeStamps; i++)
		{
			int offset = timeMultiplier + i + m_singleStepTime/m_dTime;
			finalMatrix.row(offset) = finalMatrix.row(timeMultiplier + m_singleStepTime/m_dTime - 1);
		}

		stepCount++;

		//*******left foot**********//
		if(leftSteps.rows() > (i+1))
		{
			//Rise the left foot
			groundedFoot << endStepPos;
			startingStepFootAngle = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
			startingStepFootAngle(2) += leftSteps(i, 2);
			startingStepPos << leftSteps(i, 0), leftSteps(i, 1), 0.0f, startingStepFootAngle;

			endStepFootAngle = m_vLeftFootAngleOffset;
			endStepFootAngle(2) += (leftSteps(i + 1, 2) - leftSteps(i, 2))/2 + leftSteps(i, 2);
			endStepPos << ((leftSteps(i + 1, 0) - leftSteps(i, 0))/2) + leftSteps(i, 0) + m_vLeftFootPosOffset(0),
					((leftSteps(i + 1, 1) - leftSteps(i, 1))/2) + leftSteps(i, 1) + m_vLeftFootPosOffset(1), m_stepHeight + m_vLeftFootPosOffset(2), endStepFootAngle;
			GenerateFinalMatrixForOneStep(finalMatrix, stepCount, startingStepPos, endStepPos, groundedFoot,
					m_singleStepTime/2, 0, nbSteppingTimeStamps/2, 0);

			//Lower the left foot
			startingStepPos = endStepPos;
			endStepFootAngle = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
			endStepFootAngle(2) += leftSteps(i + 1, 2);
			endStepPos << leftSteps(i + 1, 0), leftSteps(i + 1, 1), 0.0f, endStepFootAngle;
			GenerateFinalMatrixForOneStep(finalMatrix, stepCount, startingStepPos, endStepPos, groundedFoot,
					m_singleStepTime/2, nbSteppingTimeStamps/2, nbSteppingTimeStamps, 0);

			stepCount++;

			int timeMultiplier = nbSteppingTimeStamps*stepCount;
			for(int i = 0; i < nbSteppingTimeStamps; i++)
			{
				int offset = timeMultiplier + i + m_singleStepTime/m_dTime;
				finalMatrix.row(offset) = finalMatrix.row(timeMultiplier + m_singleStepTime/m_dTime - 1);
			}

			stepCount++;
		}
	}

	int offset = nbSteppingTimeStamps;
	//Add an initial state (repeat the first position for the first elements so that the zmp can be set correctly before moving the legs)
	Eigen::VectorXf initialState = finalMatrix.row(offset);
	for(int i = 0; i < offset; i++)
	{
		finalMatrix.row(i) = initialState;
	}

	return finalMatrix;
}

/** \brief Generates the final matrix for every step
 *
 * \param finalMatrix Eigen::MatrixXf&: The final matrix
 * \param stepCount int: Which step to process
 * \param startingStepPos Eigen::Vector4f&: The starting position of the moving foot (either on the ground or mid air) in x, y, z, angle
 * \param endingStepPos Eigen::Vector4f&: The ending position of the moving foot (either on the ground or mid air) in x, y, z, angle
 * \param groundedFootPos Eigen::Vector4f&: The position of the foot on the ground in x, y, z, angle
 * \param singleStepTime float: Time required to do a step
 * \param startTime int: The starting time
 * \param endTime int: The ending time
 * \param groundedFoot int: Which foot stands on the ground (1 = left, 0 = right)
 *
 */
void Trajectory::GenerateFinalMatrixForOneStep(Eigen::MatrixXf& finalMatrix, int stepCount,
		Eigen::VectorXf& startingStepPos, Eigen::VectorXf& endingStepPos, Eigen::VectorXf& groundedFootPos,
		float singleStepTime, int startTime, int endTime, int groundedFoot)
{
	Eigen::MatrixXf params(4,6);
	Eigen::VectorXf currentFootPos(6);

	//A timestamp going across all steps
	int timeMultiplier = endTime*stepCount;
	//Make sure the time multiplier is correct even if the endTime is endTime/2
	if(startTime == 0)
	{
		timeMultiplier *= 2;
	}

	/*
	//Debug
	Eigen::Vector3f startingStepPosCut;
	startingStepPosCut << startingStepPos(0), startingStepPos(1), startingStepPos(2);
	Eigen::Vector3f endingStepPosCut;
	endingStepPosCut << endingStepPos(0), endingStepPos(1), endingStepPos(2);
*/

	params = GenerateParabollicTrajParams(startingStepPos, endingStepPos, singleStepTime);

	float parabolicTrajTime = 0.0f;

	for(int time = startTime; time < endTime; time ++)
	{
		//Start the time for the parabolic traj from 0
		if(startTime != 0)
			parabolicTrajTime = m_dTime*(time - startTime);
		else
			parabolicTrajTime = m_dTime*time;

		currentFootPos = GenerateParabollicTrajectory(params, parabolicTrajTime);

		//Time
		int offset = timeMultiplier+time + m_singleStepTime/m_dTime;

		if(groundedFoot == 1)	//Left foot is 1, right foot is moving
		{
			//Right foot moving
			finalMatrix(offset, 0) = currentFootPos(0);	//x
			finalMatrix(offset, 1) = currentFootPos(1);	//y
			finalMatrix(offset, 2) = currentFootPos(2);	//z
			finalMatrix(offset, 3) = startingStepPos(3) + ((endingStepPos(3) - startingStepPos(3))/((endTime - startTime)))*(time-startTime);		//angle
			finalMatrix(offset, 4) = startingStepPos(4) + ((endingStepPos(4) - startingStepPos(4))/((endTime - startTime)))*(time-startTime);		//angle
			finalMatrix(offset, 5) = startingStepPos(5) + ((endingStepPos(5) - startingStepPos(5))/((endTime - startTime)))*(time-startTime);		//angle
			//Left foot position on the ground
			finalMatrix(offset, 6) = groundedFootPos(0);	//x
			finalMatrix(offset, 7) = groundedFootPos(1);	//y
			finalMatrix(offset, 8) = groundedFootPos(2);	//z
			finalMatrix(offset, 9) = groundedFootPos(3);	//angle
			finalMatrix(offset, 10) = groundedFootPos(4);	//angle
			finalMatrix(offset, 11) = groundedFootPos(5);	//angle
		}
		else	//Right foot is 0, left foot is moving
		{
			//Right foot position on the ground
			finalMatrix(offset, 0) = groundedFootPos(0);	//x
			finalMatrix(offset, 1) = groundedFootPos(1);	//y
			finalMatrix(offset, 2) = groundedFootPos(2);	//z
			finalMatrix(offset, 3) = groundedFootPos(3);	//angle
			finalMatrix(offset, 4) = groundedFootPos(4);	//angle
			finalMatrix(offset, 5) = groundedFootPos(5);	//angle
			//Left foot moving
			finalMatrix(offset, 6) = currentFootPos(0);	//x
			finalMatrix(offset, 7) = currentFootPos(1);	//y
			finalMatrix(offset, 8) = currentFootPos(2);	//z
			finalMatrix(offset, 9) = startingStepPos(3) + ((endingStepPos(3) - startingStepPos(3))/((endTime - startTime)))*(time-startTime);		//angle
			finalMatrix(offset, 10) = startingStepPos(4) + ((endingStepPos(4) - startingStepPos(4))/((endTime - startTime)))*(time-startTime);		//angle
			finalMatrix(offset, 11) = startingStepPos(5) + ((endingStepPos(5) - startingStepPos(5))/((endTime - startTime)))*(time-startTime);		//angle
		}

		finalMatrix(offset, 12) = groundedFoot;	// 0 = right, 1 = left foot
	}
}

/** \brief Generates the cubic trajectory params
 *
 * \param initialPos Eigen::Vector4f: The initial position
 * \param finalPos Eigen::Vector4f: The final position
 * \param stepTimeLapse float: The time required to perform a step
 *
 */
Eigen::MatrixXf Trajectory::GenerateParabollicTrajParams(Eigen::VectorXf initialPos, Eigen::VectorXf finalPos, float stepTimeLapse)
{
	Eigen::MatrixXf params(4,initialPos.rows());

	//Calculate params for x,y,z
	for(int i = 0; i < initialPos.rows(); i++)
	{
		params(0, i) = 2*(initialPos(i)-finalPos(i))/(pow(stepTimeLapse,3));
		params(1, i) = -(1.5f)*stepTimeLapse*params(0,i);
		params(2, i) = 0;
		params(3, i) = initialPos(i);
	}

	return params;
}


/** \brief Returns the position (x,y,z) of a foot at a certain time following a parabollic trajectory
 *
 * \param params Eigen::MatrixXf: The trajectory parameters
 * \param currentTime float: A given time
 *
 */
Eigen::VectorXf Trajectory::GenerateParabollicTrajectory(Eigen::MatrixXf params, float currentTime)
{
	Eigen::VectorXf trajectory(params.cols());

	for(int i = 0; i < params.cols(); i++)
	{
		trajectory(i) = params(0,i)*(pow(currentTime,3)) + params(1,i)*(pow(currentTime,2)) + params(2,i)*currentTime + params(3,i);
	}

	return trajectory;
}

/** \brief Generates the ZMP position to follow the steps
 *
 * \param rightSteps Eigen::MatrixXf: The right foot steps
 * \param leftSteps Eigen::MatrixXf: The left foot steps
 *
 */
Eigen::MatrixXf Trajectory::GenerateZMP(Eigen::MatrixXf rightSteps, Eigen::MatrixXf leftSteps, int finalMatrixSize)
{
	Eigen::Vector3f rightPelvisAngleOffset;
	rightPelvisAngleOffset(0) = -m_vRightPelvisAngleOffset(2);
	rightPelvisAngleOffset(1) = m_vRightPelvisAngleOffset(0);
	rightPelvisAngleOffset(2) = m_vRightPelvisAngleOffset(1);

	Eigen::Vector3f leftPelvisAngleOffset;
	leftPelvisAngleOffset(0) = -m_vLeftPelvisAngleOffset(2);
	leftPelvisAngleOffset(1) = -m_vLeftPelvisAngleOffset(0);
	leftPelvisAngleOffset(2) = -m_vLeftPelvisAngleOffset(1);

	Eigen::MatrixXf trajectory(finalMatrixSize, 6);

	rightSteps.conservativeResize(Eigen::NoChange, 2);
	leftSteps.conservativeResize(Eigen::NoChange, 2);

	Eigen::VectorXf initialPoint(6);
	initialPoint << ((leftSteps.row(0) + rightSteps.row(0))/2).transpose(), m_ZMPHeight, Eigen::Vector3f(0.0f, 0.0f, 0.0f);
	Eigen::VectorXf finalPoint(6);
	finalPoint << (leftSteps.row(0)).transpose(), m_ZMPHeight, Eigen::Vector3f(0.0f, 0.0f, 0.0f);
	finalPoint(0) = finalPoint(0) + m_vRightPelvisPosOffset(0);
	finalPoint(1) = finalPoint(1) + m_vRightPelvisPosOffset(1);
	finalPoint(2) = finalPoint(2) + m_vRightPelvisPosOffset(2);

	//Trajectory from point A to left footprint
	GenerateZMPStepTransfer(trajectory, initialPoint, finalPoint, 0, rightPelvisAngleOffset);

	int stepIndex = 2;
    for(int i =0, j = 1; i < leftSteps.rows() - 1; ++i, ++j)
    {
		//Trajectory from left to right to left foot steps
    	initialPoint(0) = leftSteps(i, 0);
    	initialPoint(1) = leftSteps(i, 1);
    	finalPoint(0) = rightSteps(j, 0) + m_vLeftPelvisPosOffset(0);
    	finalPoint(1) = rightSteps(j, 1) + m_vLeftPelvisPosOffset(1);
    	GenerateZMPStepTransfer(trajectory, initialPoint, finalPoint, stepIndex, leftPelvisAngleOffset);
    	stepIndex+=2;
    	if(leftSteps.rows() > i + 1)
    	{
    		initialPoint = finalPoint;
        	finalPoint(0) = leftSteps(i+1, 0) + m_vRightPelvisPosOffset(0);
        	finalPoint(1) = leftSteps(i+1, 1) + m_vRightPelvisPosOffset(1);
    		GenerateZMPStepTransfer(trajectory, initialPoint, finalPoint, stepIndex, rightPelvisAngleOffset);
    		stepIndex+=2;
    	}

    }

    //Append the last step (left foot) to pointD
    Eigen::VectorXf pointD(6);
    pointD << ((leftSteps.row(leftSteps.rows() - 1) + rightSteps.row(rightSteps.rows() - 1))/2).transpose(), m_ZMPHeight, Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    initialPoint(0) = leftSteps(leftSteps.rows()-1, 0);
    initialPoint(1) = leftSteps(leftSteps.rows()-1, 1);
    GenerateZMPStepTransfer(trajectory, initialPoint, pointD, stepIndex, Eigen::Vector3f(0.0f, 0.0f, 0.0f));

    return trajectory;
}

void Trajectory::GenerateZMPStepTransfer(Eigen::MatrixXf& trajectoryMatrix, Eigen::VectorXf startingPos, Eigen::VectorXf endingPos, int stepIndex, Eigen::Vector3f pelvisAngleOffset)
{
	Eigen::MatrixXf params(4,6);
	float stepTime = m_singleStepTime/m_dTime;
	params = GenerateParabollicTrajParams(startingPos, endingPos, m_singleStepTime);
	for(int time = 0; time < stepTime; time ++)
	{
		trajectoryMatrix.row(time + stepIndex*stepTime) = (GenerateParabollicTrajectory(params, time*m_dTime)).transpose();
	}

	if(stepTime + stepTime*stepIndex < trajectoryMatrix.rows())
	{
		//Add a "step" where there is no zmp movement to allow the foot to move freely
		startingPos = endingPos;
		endingPos(3) = pelvisAngleOffset(0);
		endingPos(4) = pelvisAngleOffset(1);
		endingPos(5) = pelvisAngleOffset(2);
		params = GenerateParabollicTrajParams(startingPos, endingPos, m_singleStepTime/2);
		//While foot is rising
		int initialTime = stepTime + stepTime*stepIndex;
		for(int time = initialTime; time < (stepTime*stepIndex + stepTime*1.5); time++)
		{
			trajectoryMatrix.row(time) = (GenerateParabollicTrajectory(params, (time - initialTime)*m_dTime)).transpose();
		}
		//While foot is lowering
		params = GenerateParabollicTrajParams(endingPos, startingPos, m_singleStepTime/2);
		initialTime = stepTime*stepIndex + stepTime*1.5;
		for(int time = initialTime; time < (stepTime*stepIndex + stepTime*2); time++)
		{
			trajectoryMatrix.row(time) = (GenerateParabollicTrajectory(params, (time - initialTime)*m_dTime)).transpose();
		}

		/*for(int time = stepTime + stepTime*stepIndex; time < stepTime*stepIndex + stepTime*2; time++)
		{
			trajectoryMatrix.row(time) = trajectoryMatrix.row(time-1);
		}*/
	}
}

/** \brief Generates the center of mass position to follow the steps
 *
 * \param zmpMatrix Eigen::MatrixXf: The zmp matrix
 *
 */
Eigen::MatrixXf Trajectory::GenerateCOM(Eigen::MatrixXf zmpMatrix)
{
	int nbForwardChecks = 500;

	//ZMP Riccati
	Eigen::Matrix3f A;

	Eigen::Vector3f b0(0.0f, 0.0f, m_dTime);

	Eigen::Vector3f c0(0.0f, 0.0f, 1.0f);

	Eigen::Vector3f x0(0.0f, 0.0f, 0.0f);

	Eigen::MatrixXf f(1, nbForwardChecks);

	string line;

	string filename = "config/zmp.txt";
	//ifstream ifs (filename);

	std::ifstream ifs;
    ifs.open (filename.c_str(), std::ifstream::in);

	if (ifs.is_open())
	{
		//zmp height
		getline(ifs,line);

		//newline
		getline(ifs,line);

		//get A
		int index = 0;
		for(int i = 0; i < 9; i++)
		{
			index = i/3;
			getline(ifs,line);
			A(i - index*3, index) = float(atof(line.c_str()));
		}

		//newline
		getline(ifs,line);

		//Get f
		for (int i = 0; i < nbForwardChecks; i++)
		{
			getline(ifs, line);
			f(i) = float(atof(line.c_str()));
		}
		ifs.close();
	}
	else
	{
		printf("Cannot open zmp file.");
		throw;
	}

	Eigen::MatrixXf xyZmp(zmpMatrix.rows()+nbForwardChecks,2);

	//Add the last point nbForwardChecks times
	xyZmp << zmpMatrix.col(0), zmpMatrix.col(1), Eigen::VectorXf::Constant(nbForwardChecks, zmpMatrix(zmpMatrix.rows()-1, 0)), Eigen::VectorXf::Constant(nbForwardChecks, zmpMatrix(zmpMatrix.rows()-1, 1));

	Eigen::MatrixXf xkVector(3, zmpMatrix.rows());
	Eigen::Vector3f xkAnterieur = x0;
	xkAnterieur = xkAnterieur.transpose();
	Eigen::Vector3f xk;

	Eigen::MatrixXf ykVector(3, zmpMatrix.rows());
	Eigen::Vector3f ykAnterieur = x0;
	Eigen::Vector3f yk;

	b0 = b0.transpose();

	//Loop for x and y
	for(int i = 0; i < xyZmp.rows() - nbForwardChecks; i++)
	{
		xk = A*xkAnterieur + b0*f*xyZmp.block(i+1, 0,nbForwardChecks, 1);
		yk = A*ykAnterieur + b0*f*xyZmp.block(i+1, 1,nbForwardChecks, 1);

		xkAnterieur = xk;
		xkVector.col(i) = xk;
		ykAnterieur = yk;
		ykVector.col(i) = yk;
	}

	Eigen::VectorXf XCom = xkVector.row(0);
	Eigen::VectorXf YCom = ykVector.row(0);

	Eigen::MatrixXf COM(XCom.size(), 3);
	COM << XCom, YCom, Eigen::VectorXf::Constant(XCom.size(), m_ZMPHeight), zmpMatrix.col(3), zmpMatrix.col(4), zmpMatrix.col(5);

	return COM;
}
