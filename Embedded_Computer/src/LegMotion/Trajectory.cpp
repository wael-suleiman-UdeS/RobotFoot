#include "Trajectory.h"

#include "EigenUtils.h"
#include <vector>
#include <math.h>


Trajectory::Trajectory()
: m_samplingTime(0.005f)
, m_singleStepTime(2.0f)
, m_dLeg(0.1f)
, m_dStep(0.3f)
, m_nbTrajectoryPoints(101)
{}

Trajectory::~Trajectory()
{}

void Trajectory::GenerateWalk(Eigen::Vector2f startingPoint, Eigen::Vector2f goalPoint, float goalAngle, float startingAngle)
{
	Eigen::VectorXf xTrajectory(m_nbTrajectoryPoints);
	Eigen::VectorXf yTrajectory(m_nbTrajectoryPoints);

	//2nd degree Bezier curve
	BezierDegre2(xTrajectory, yTrajectory, startingPoint, goalPoint, startingAngle, goalAngle);

	Eigen::VectorXf xInner(xTrajectory.innerSize());
	Eigen::VectorXf yInner(yTrajectory.innerSize());
	Eigen::VectorXf xOuter(xTrajectory.innerSize());
	Eigen::VectorXf yOuter(yTrajectory.innerSize());

	//Parallel curve
	ParallelCurve(xInner, yInner, xOuter, yOuter, xTrajectory, yTrajectory);

	int stepMatrixSize = xInner.rows() + 1;
	Eigen::MatrixXf rightSteps(stepMatrixSize/2 + 2, 2);
	Eigen::MatrixXf leftSteps(stepMatrixSize/2 + 1, 2);

	//Generate steps on the ground to follow the parallel curve
	GenerateSteps(rightSteps, leftSteps, xInner, yInner, xOuter, yOuter, startingPoint, startingAngle);

	//Create trajectory for moving foot
	GenerateStepsTrajectories();

	//Append all matrix

	//Append time vector


	/*
	//Create Ttn
    Eigen::VectorXf ttn = Eigen::VectorXf::LinSpaced(totalTime/tEch, 0, totalTime);
	*/

}

//enum which leg to use?
void Trajectory::GenerateKick()
{

}

//This function might be removed, this should be done when the steps are calculated
Eigen::MatrixXf Trajectory::ToList(Eigen::Vector2f pointA, Eigen::Vector2f pointD, Eigen::MatrixXf leftTrajectory, Eigen::MatrixXf rightTrajectory)
{ 
    //Take the longest trajectory
    int leftTrajLength = leftTrajectory.innerSize();
    int rightTrajLength = rightTrajectory.innerSize();
    int largestTrajLength = leftTrajLength >= rightTrajLength ? leftTrajLength : rightTrajLength;

    Eigen::MatrixXf result(leftTrajLength+rightTrajLength, 2);

    int matrixIndex = 0;
    for(int i = 0; i < largestTrajLength; ++i)
    {
        if(i < rightTrajLength)
        {
            result(matrixIndex,0) = rightTrajectory(i,0);
            result(matrixIndex,1) = rightTrajectory(i,1);
        }
        if(i < leftTrajLength)
        {
            result(matrixIndex+1,0) = leftTrajectory(i,0);
            result(matrixIndex+1,1) = leftTrajectory(i,1);
        }
        matrixIndex +=2;
    }

    result(0,0) = pointA(0);
    result(0,1) = pointA(1);

    result(largestTrajLength,0) = pointD(0);
    result(largestTrajLength,1) = pointD(1);

    return result;
}

/** \brief Generate a trajectory for a 2nd degree Bezier curve
 *
 * \param xTrajectory Eigen::VectorXf&: Points in X for the trajectory
 * \param yTrajectory Eigen::VectorXf&: Points in Y for the trajectory
 * \param pointA Eigen::Vector2f: Starting point
 * \param pointD Eigen::Vector2f: Ending point
 * \param startAngle int: Starting angle
 * \param endAngle int: Ending angle
 * \param dist int: optional parameter, distance
 *
 */
void Trajectory::BezierDegre2(Eigen::VectorXf& xTrajectory, Eigen::VectorXf& yTrajectory,
	Eigen::Vector2f pointA, Eigen::Vector2f pointD, int startAngle, int endAngle, int dist)
{
	Eigen::Vector2f pointB(
		pointA(0) + dist*cos(startAngle),
		pointA(1) + dist*sin(startAngle));

	Eigen::Vector2f pointC(
		pointD(0) - dist*cos(endAngle),
		pointD(1) - dist*sin(endAngle));

	//t varie entre 0 et 1. echantillonage (plus "t" est petit, plus la courbe est lisse)
	for(int i = 0; i < m_nbTrajectoryPoints; ++i)
	{
		float t = i*0.01;
		xTrajectory(i) = pointA(0)*pow(1-t,3) + 3*pointB(0)*t*pow(1-t,2) + 3*pointC(0)*pow(t,2)*(1-t) + pointD(0)*pow(t,3);
		yTrajectory(i) = pointA(1)*pow(1-t,3) + 3*pointB(1)*t*pow(1-t,2) + 3*pointC(1)*pow(t,2)*(1-t) + pointD(1)*pow(t,3);
	}
}

// d=10;           % (meters) distance from curve to the parallel curve
//             	% default is d=0.1;
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

	Eigen::MatrixXf unv = EigenUtils::AppendMatrixColumn(dy, -dx);

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

void Trajectory::GenerateSteps(Eigen::MatrixXf &rightSteps, Eigen::MatrixXf &leftSteps, Eigen::VectorXf &xInner, Eigen::VectorXf &yInner, Eigen::VectorXf &xOuter, Eigen::VectorXf &yOuter,
		Eigen::Vector2f startingPoint, int startAngle)
{
	Eigen::Vector2f currentLeftStepPos;
	currentLeftStepPos(0) = startingPoint(0) - (m_dLeg*sin(startAngle));
	currentLeftStepPos(1) = startingPoint(1) + (m_dLeg*cos(startAngle));

	Eigen::Vector2f currentRightStepPos;
	currentRightStepPos(0) = startingPoint(0) + (m_dLeg*sin(startAngle));
	currentRightStepPos(1) = startingPoint(1) - (m_dLeg*cos(startAngle));

	Eigen::MatrixXf trajL(xInner.rows() + 1, 2);
	Eigen::MatrixXf trajR(xOuter.rows() + 1, 2);

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

			currentRightStepPos(0) = xOuter(i);
			currentRightStepPos(1) = yOuter(i);

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
		trajR.resize(trajIndex + 1, 2);
	}
	else
	{
		//Set the right size for trajR
		trajR.resize(trajIndex, 2);
	}

	int xInnerLength = xInner.rows() - 1;
	int yInnerLength = yInner.rows() - 1;
	if(xInner(xInnerLength) != trajL(trajIndex - 1, 0) && yInner(yInnerLength) != trajL(trajIndex - 1, 1))
	{
		trajL(trajIndex, 0) = xInner(xInnerLength);
		trajL(trajIndex, 1) = yInner(yInnerLength);

		//Set the right size for trajL
		trajL.resize(trajIndex + 1, 2);
	}
	else
	{
		//Set the right size for trajL
		trajL.resize(trajIndex, 2);
	}

	//Create arrays for right and left steps

	int trajLSize = trajL.rows();

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
	rightSteps.row(trajLSize - 1) = trajR.row(trajLSize - 1);
	leftSteps.row(trajLSize - 1) = trajL.row(trajLSize - 1);
}
