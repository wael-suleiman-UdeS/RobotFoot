#include "Trajectory.h"

#include "EigenUtils.h"
#include <vector>

Trajectory::Trajectory()
: samplingTime(0.005f)
, singleStepTime(2.0f)
{}

Trajectory::~Trajectory()
{}

void Trajectory::GenerateWalk(Eigen::Vector2f startingPoint, Eigen::Vector2f goalPoint, float goalAngle, float startingAngle)
{
	Eigen::VectorXf xTrajectory;
	Eigen::VectorXf yTrajectory;

	//2nd degree Bezier curve
	BezierDegre2(xTrajectory, yTrajectory, startingPoint, goalPoint, startingAngle, goalAngle);

	Eigen::VectorXf xInner(xTrajectory.innerSize());
	Eigen::VectorXf yInner(yTrajectory.innerSize());
	Eigen::VectorXf xOuter(xTrajectory.innerSize());
	Eigen::VectorXf yOuter(yTrajectory.innerSize());

	//Parallel curve
	ParallelCurve(xInner, yInner, xOuter, yOuter, xTrajectory, yTrajectory);

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
	for(float t = 0; t < 1; t += 0.01)
	{
		xTrajectory(t*100) = pointA(0)*pow(1-t,3) + 3*pointB(0)*t*pow(1-t,2) + 3*pointC(0)*pow(t,2)*(1-t) + pointD(0)*pow(t,3);
		yTrajectory(t*100) = pointA(1)*pow(1-t,3) + 3*pointB(1)*t*pow(1-t,2) + 3*pointC(1)*pow(t,2)*(1-t) + pointD(1)*pow(t,3);
	}
}

// d=10;           % (meters) distance from curve to the parallel curve
//             	% default is d=0.1;
void Trajectory::ParallelCurve(Eigen::VectorXf &xInner, Eigen::VectorXf &yInner, Eigen::VectorXf &xOuter, Eigen::VectorXf &yOuter, Eigen::VectorXf& x, Eigen::VectorXf& y, float d)
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
	for(int i = 0; i < dy.rows(); i ++)
	{
		norm = unv.row(i).norm();
		unv.row(i) /= norm;
	}

	for(int i = 0; i < x.innerSize() - 1; ++i)
	{
		xInner(i) = x(i) - unv(i, 0)*d;
		yInner(i) = y(i) - unv(i, 1)*d;

		xOuter(i) = x(i) + unv(i, 0)*d;
		yOuter(i) = y(i) + unv(i, 1)*d;
	}
}
