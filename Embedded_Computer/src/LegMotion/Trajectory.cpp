#include "Trajectory.h"

#include "EigenUtils.h"

#include <math.h>

#include "../../ThirdParty/Eigen/Dense"


/** \brief Constructor
 *
 *
 */
Trajectory::Trajectory()
: m_singleStepTime(1.0f)
, m_dLeg(0.1f)
, m_dStep(0.3f)
, m_stepHeight(0.02f)
, m_dTime(0.01f)
, m_nbTrajectoryPoints(101)
{}

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
Eigen::MatrixXf Trajectory::GenerateWalk(Eigen::Vector2f startingPoint, Eigen::Vector2f goalPoint, Eigen::Vector2f goalAngle, Eigen::Vector2f startingAngle, float stepTime, float stepHeight)
{
	m_ZMPHeight = 0.20f;
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
	int finalMatrixSize = (m_singleStepTime/m_dTime)*(rightSteps.rows() + leftSteps.rows());
	Eigen::MatrixXf zmp = GenerateZMP(rightSteps, leftSteps);

	//Create trajectory for moving foot
	Eigen::MatrixXf trajectoryMatrix = GenerateParabollicStepsTrajectories(rightSteps, leftSteps, finalMatrixSize);

	//Create a time vector
	Eigen::VectorXf time = Eigen::VectorXf::LinSpaced(finalMatrixSize, 0, finalMatrixSize*m_dTime);

	//Append ZMP to final matrix
	Eigen::MatrixXf finalMatrix(finalMatrixSize, 13);
	finalMatrix << time, trajectoryMatrix, zmp;

	return finalMatrix;
}

/** \brief Generates a matrix which contains the necessary information to perform a kick
 *
 */
void Trajectory::GenerateKick()
{

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
		if(groundedFoot)
			currentRightFootPos = GenerateParabollicTrajectory(paramsRightFoot, m_dTime*time);
		else
			currentLeftFootPos = GenerateParabollicTrajectory(paramsLeftFoot, m_dTime*time);
		currentPelvisPos = GenerateParabollicTrajectory(paramsPelvis, m_dTime*time);

		if(groundedFoot == 1)	//Left foot is 1, right foot is moving
		{
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
	Eigen::Vector2f pointA, Eigen::Vector2f pointD, Eigen::Vector2f startAngle, Eigen::Vector2f endAngle, int dist)
{
	Eigen::Vector2f pointB(
		pointA(0) + dist*cos(startAngle(0)),
		pointA(1) + dist*sin(startAngle(1)));

	Eigen::Vector2f pointC(
		pointD(0) - dist*cos(endAngle(0)),
		pointD(1) - dist*sin(endAngle(1)));

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
		xInner(i) = x(i) - unv(i, 0)*m_dLeg;
		yInner(i) = y(i) - unv(i, 1)*m_dLeg;

		xOuter(i) = x(i) + unv(i, 0)*m_dLeg;
		yOuter(i) = y(i) + unv(i, 1)*m_dLeg;
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
	currentLeftStepPos(0) = startingPoint(0) - (m_dLeg*sin(startAngle(0)));
	currentLeftStepPos(1) = startingPoint(1) + (m_dLeg*cos(startAngle(1)));
	currentLeftStepPos(2) = angles(0);

	Eigen::Vector3f currentRightStepPos;
	currentRightStepPos(0) = startingPoint(0) + (m_dLeg*sin(startAngle(0)));
	currentRightStepPos(1) = startingPoint(1) - (m_dLeg*cos(startAngle(1)));
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
	for(int i = 1; i < trajLSize - 1; i++)
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
	Eigen::Vector3f stepPosition;

	//Final matrix:
	//rightStepX, rightStepY, rightStepZ, rightStepAngle, leftStepX, leftStepY, leftStepZ, leftStepAngle, GroundedFoot(0 = right, 1 = left)
	Eigen::MatrixXf finalMatrix(finalMatrixSize, 9);
	finalMatrix = Eigen::MatrixXf::Zero(finalMatrixSize, 9);

	Eigen::Vector4f groundedFoot;
	Eigen::Vector4f startingStepPos;
	Eigen::Vector4f endStepPos;

	int nbSteppingTimeStamps (m_singleStepTime/m_dTime);

	int stepCount = 0;
	for(int i = 0; i < rightSteps.rows() - 1; i++)
	{
		//*******right foot**********//
		//Rise the right foot
		groundedFoot << leftSteps(i, 0), leftSteps(i, 1), 0.0f, leftSteps(i, 2);
		startingStepPos << rightSteps(i, 0), rightSteps(i, 1), 0.0f, rightSteps(i, 2);
		endStepPos << ((rightSteps(i + 1, 0) - rightSteps(i, 0))/2) + rightSteps(i, 0),
				((rightSteps(i + 1, 1) - rightSteps(i, 1))/2) + rightSteps(i, 1), m_stepHeight, (rightSteps(i + 1, 2) - rightSteps(i, 2))/2 + rightSteps(i, 2);

		GenerateFinalMatrixForOneStep(finalMatrix, stepCount, startingStepPos, endStepPos, groundedFoot,
				m_singleStepTime/2, 0, nbSteppingTimeStamps/2, 1);

		//Lower the right foot
		startingStepPos = endStepPos;
		endStepPos << rightSteps(i + 1, 0), rightSteps(i + 1, 1), 0.0f, rightSteps(i + 1, 2);
		GenerateFinalMatrixForOneStep(finalMatrix, stepCount, startingStepPos, endStepPos, groundedFoot,
				m_singleStepTime/2, nbSteppingTimeStamps/2, nbSteppingTimeStamps, 1);

		stepCount++;

		//*******left foot**********//
		if(leftSteps.rows() > (i+1))
		{
			//Rise the left foot
			groundedFoot << rightSteps(i + 1, 0), rightSteps(i + 1, 1), 0.0f, rightSteps(i + 1, 2);
			startingStepPos << leftSteps(i, 0), leftSteps(i, 1), 0.0f, leftSteps(i, 2);
			endStepPos << ((leftSteps(i + 1, 0) - leftSteps(i, 0))/2) + leftSteps(i, 0),
					((leftSteps(i + 1, 1) - leftSteps(i, 1))/2) + leftSteps(i, 1), m_stepHeight, (leftSteps(i + 1, 2) - leftSteps(i, 2))/2 + leftSteps(i, 2);
			GenerateFinalMatrixForOneStep(finalMatrix, stepCount, startingStepPos, endStepPos, groundedFoot,
					m_singleStepTime/2, 0, nbSteppingTimeStamps/2, 0);

			//Lower the left foot
			startingStepPos = endStepPos;
			endStepPos << leftSteps(i + 1, 0), leftSteps(i + 1, 1), 0.0f, leftSteps(i + 1, 2);
			GenerateFinalMatrixForOneStep(finalMatrix, stepCount, startingStepPos, endStepPos, groundedFoot,
					m_singleStepTime/2, nbSteppingTimeStamps/2, nbSteppingTimeStamps, 0);

			stepCount++;
		}
	}

	int offset = 1/m_dTime;
	//Add an initial state (repeat the first position for the first elements so that the zmp can be set correctly before moving the legs)
	Eigen::VectorXf initialState = finalMatrix.row(offset);
	Eigen::VectorXf finalState = finalMatrix.row(finalMatrixSize - offset - 1);
	for(int i = 0; i < offset; i++)
	{
		finalMatrix.row(i) = initialState;
		finalMatrix.row(finalMatrixSize-1-i) = finalState;
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
		Eigen::Vector4f& startingStepPos, Eigen::Vector4f& endingStepPos, Eigen::Vector4f& groundedFootPos,
		float singleStepTime, int startTime, int endTime, int groundedFoot)
{
	Eigen::MatrixXf params(4,3);
	Eigen::Vector3f currentFootPos;

	//A timestamp going across all steps
	int timeMultiplier = endTime*stepCount;
	//Make sure the time multiplier is correct even if the endTime is endTime/2
	if(startTime == 0)
	{
		timeMultiplier *= 2;
	}

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
		int offset = timeMultiplier+time + 1/m_dTime;

		if(groundedFoot == 1)	//Left foot is 1, right foot is moving
		{
			//Right foot moving
			finalMatrix(offset, 0) = currentFootPos(0);	//x
			finalMatrix(offset, 1) = currentFootPos(1);	//y
			finalMatrix(offset, 2) = currentFootPos(2);	//z
			finalMatrix(offset, 3) = startingStepPos(3) + ((endingStepPos(3) - startingStepPos(3))/((endTime - startTime)))*(time-startTime);		//angle
			//Left foot position on the ground
			finalMatrix(offset, 4) = groundedFootPos(0);	//x
			finalMatrix(offset, 5) = groundedFootPos(1);	//y
			finalMatrix(offset, 6) = groundedFootPos(2);	//z
			finalMatrix(offset, 7) = groundedFootPos(3);	//angle
		}
		else	//Right foot is 0, left foot is moving
		{
			//Right foot position on the ground
			finalMatrix(offset, 0) = groundedFootPos(0);	//x
			finalMatrix(offset, 1) = groundedFootPos(1);	//y
			finalMatrix(offset, 2) = groundedFootPos(2);	//z
			finalMatrix(offset, 3) = groundedFootPos(3);	//angle
			//Left foot moving
			finalMatrix(offset, 4) = currentFootPos(0);	//x
			finalMatrix(offset, 5) = currentFootPos(1);	//y
			finalMatrix(offset, 6) = currentFootPos(2);	//z
			finalMatrix(offset, 7) = startingStepPos(3) + ((endingStepPos(3) - startingStepPos(3))/((endTime - startTime)))*(time-startTime);		//angle
		}

		finalMatrix(timeMultiplier+time, 8) = groundedFoot;	// 0 = right, 1 = left foot
	}
}

/** \brief Generates the cubic trajectory params
 *
 * \param initialPos Eigen::Vector4f: The initial position
 * \param finalPos Eigen::Vector4f: The final position
 * \param stepTimeLapse float: The time required to perform a step
 *
 */
Eigen::MatrixXf Trajectory::GenerateParabollicTrajParams(Eigen::Vector4f initialPos, Eigen::Vector4f finalPos, float stepTimeLapse)
{
	Eigen::MatrixXf params(4,3);

	//Calculate params for x,y,z
	for(int i = 0; i < initialPos.rows()-1; i++)
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
Eigen::Vector3f Trajectory::GenerateParabollicTrajectory(Eigen::MatrixXf params, float currentTime)
{
	Eigen::Vector3f trajectory;

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
 * \param finalMatrixSize int: The final matrix size
 *
 */
Eigen::MatrixXf Trajectory::GenerateZMP(Eigen::MatrixXf rightSteps, Eigen::MatrixXf leftSteps)
{
	rightSteps.conservativeResize(Eigen::NoChange, 2);
	leftSteps.conservativeResize(Eigen::NoChange, 2);

	Eigen::Vector2f initialPoint = (leftSteps.row(0) + rightSteps.row(0))/2;

	//Trajectory from point A to left footprint
    Eigen::MatrixXf trajectory = EigenUtils::MXB(initialPoint, leftSteps.row(0), m_dTime);

    for(int i =0, j = 1; i < rightSteps.rows() - 1; ++i, ++j)
    {
		//Trajectory from left to right to left foot steps
		Eigen::MatrixXf mxbMatrix = EigenUtils::CreateCombinedMXBMatrix(leftSteps, rightSteps, m_dTime, i, j);

        Eigen::MatrixXf tempMatrix(trajectory.rows()+mxbMatrix.rows(), trajectory.cols());
        tempMatrix << trajectory, mxbMatrix;
        trajectory.swap(tempMatrix);
    }

    //Append the last step (left foot) to pointD
    Eigen::Vector2f finalPoint = (leftSteps.row(leftSteps.rows() - 1) + rightSteps.row(rightSteps.rows() - 1))/2;
    Eigen::MatrixXf finalStepTraj = EigenUtils::MXB(leftSteps.row(leftSteps.rows()-1), finalPoint, m_dTime);

    Eigen::MatrixXf tempMatrix(trajectory.rows()+finalStepTraj.rows(), trajectory.cols() + 1);
    tempMatrix << trajectory, Eigen::VectorXf::Constant(trajectory.rows(), m_ZMPHeight), finalStepTraj, Eigen::VectorXf::Constant(finalStepTraj.rows(), m_ZMPHeight);

    trajectory.swap(tempMatrix);

    return trajectory;
}
