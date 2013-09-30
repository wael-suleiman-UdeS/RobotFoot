#include "Trajectory.h"
#include <vector>

Trajectory::Trajectory()
{}

Trajectory::~Trajectory()
{}

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

Eigen::MatrixXf Trajectory::ToList(Eigen::Vector2f pointA, Eigen::Vector2f pointD, Eigen::MatrixXf leftTrajectory, Eigen::MatrixXf rightTrajectory)
{ 
    //Take the longest trajectory
    int leftTrajLength = leftTrajectory.innerSize();
    int rightTrajLength = rightTrajectory.innerSize();
    int largestTrajLength = leftTrajLength >= rightTrajLength ? leftTrajLength : rightTrajLength;

    Eigen::MatrixXf result(leftTrajLength+rightTrajLength+2, 2);

    result(0,0) = pointA(0);
    result(0,1) = pointA(1);

    int matrixIndex = 1;
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

    result(largestTrajLength+1,0) = pointD(0);
    result(largestTrajLength+1,1) = pointD(1);

    return result;
}

//Exemple de magn.m
//float testNorm = leftTraj.col(0).norm();
//float testNorm = leftTraj.col(1).norm();
//le faire pour chaque colonne qu'on veut (x,y)

Eigen::MatrixXf Trajectory::MXB(Eigen::Vector2f pointA, Eigen::Vector2f pointB, float increment, int offset)
{   
    int matrixSize = 1/increment;
    Eigen::MatrixXf result(matrixSize, 2);
    int matrixIndex = 0;
    for(float t = offset*increment; matrixIndex < matrixSize ; t += increment, matrixIndex++)
    {        
        result(matrixIndex,0) = pointA(0) + t * (pointB(0) - pointA(0));
        result(matrixIndex,1) = pointA(1) + t * (pointB(1) - pointA(1));
    }

    return result;
}

Eigen::MatrixXf Trajectory::SpatialZMP(Eigen::Vector2f pointA, Eigen::Vector2f pointD, Eigen::MatrixXf leftTrajectory, Eigen::MatrixXf rightTrajectory, float increment)
{
    Eigen::MatrixXf trajectory = MXB(pointA, leftTrajectory.row(0), increment);

    for(int i =0, j = 1; i < leftTrajectory.innerSize(); ++i, ++j)
    {
		Eigen::MatrixXf mxbMatrix = CreateCombinedMXBMatrix(leftTrajectory, rightTrajectory, increment, i, j);

        Eigen::MatrixXf tempMatrix = AppendMatrixRow(trajectory, mxbMatrix);
        trajectory.swap(tempMatrix);
    }

    //Append the last step to pointD
    Eigen::MatrixXf finalStepTraj = MXB(leftTrajectory.row(leftTrajectory.rows()-1), pointD, increment);
    Eigen::MatrixXf tempMatrix = AppendMatrixRow(trajectory, finalStepTraj);
    trajectory.swap(tempMatrix);

    return trajectory;
}

Eigen::MatrixXf Trajectory::AppendMatrixRow(Eigen::MatrixXf matrixA, Eigen::MatrixXf matrixB)
{
    Eigen::MatrixXf appendedMatrix(matrixA.rows()+matrixB.rows(), matrixA.cols());
    appendedMatrix << matrixA, matrixB;

    return appendedMatrix;
}

Eigen::MatrixXf Trajectory::AppendMatrixColumn(Eigen::MatrixXf matrixA, Eigen::MatrixXf matrixB)
{
    Eigen::MatrixXf appendedMatrix(matrixA.rows(), matrixA.cols()+matrixB.cols());
    appendedMatrix << matrixA, matrixB;

    return appendedMatrix;
}

Eigen::MatrixXf Trajectory::CreateCombinedMXBMatrix(Eigen::MatrixXf matrixA, Eigen::MatrixXf matrixB, float increment, int i, int j, int offset)
{
    //Create a matrix with right to left and left to right zmp trajectory
    Eigen::MatrixXf mxbMatrixBA = MXB(matrixA.row(i), matrixB.row(j), increment);
    
    //Append both step trajectories 
    Eigen::MatrixXf mxbMatrix;
    if(matrixA.rows() > i+1)
    {  
        Eigen::MatrixXf mxbMatrixAB = MXB(matrixB.row(j), matrixA.row(i+1), increment);
        mxbMatrix = AppendMatrixRow(mxbMatrixBA, mxbMatrixAB);  
    }
    else
    {
        mxbMatrix = mxbMatrixBA;
    }

	return mxbMatrix;
}

//Mettre pointeurs
Eigen::MatrixXf Trajectory::TemporalZMP(Eigen::Vector2f pointA, Eigen::Vector2f pointD, Eigen::MatrixXf leftTrajectory, Eigen::MatrixXf rightTrajectory, int tp, float tEch)
{
    float ts = 0.2*tp;

    //Create Tpn
    Eigen::VectorXf tpn = Eigen::VectorXf::LinSpaced(tp/tEch - 1, tEch, tp);

    //Create Tsn
    Eigen::VectorXf tsn = tpn;
    tsn.resize(tpn.rows()/5);

    int trajectorySize = (leftTrajectory.innerSize() + rightTrajectory.innerSize() +1);

    float TempsTotal = trajectorySize * (ts+tp);
    Eigen::Vector3f tempDeplacement(pointA(0), pointA(1), 0);//(X,Y,T)
	
	Eigen::MatrixXf trajectory = MXB(pointA, leftTrajectory.row(0), tEch/ts);

    for(int i =0, j = 1; i < leftTrajectory.innerSize(); ++i, ++j)
    {
		Eigen::MatrixXf mxbMatrix = CreateCombinedMXBMatrix(leftTrajectory.row, rightTrajectory.row, tEch/ts, i, j, 1);
		Eigen::MatrixXf deplacementZMP = AppendMatrixColumn(mxbMatrix, tsn);
	
		Eigen::VectorXf xAttendRight(tp/tEch) = Eigen::VectorXf::Constant(rightTrajectory(j, 0));//manque possiblement un step a la fin
		Eigen::VectorXf yAttendRight(tp/tEch) = Eigen::VectorXf::Constant(rightTrajectory(j, 1));

		Eigen::VectorXf xAttendLeft(tp/tEch) = Eigen::VectorXf::Constant(leftTrajectory(i, 0));//manque possiblement un step a la fin
		Eigen::VectorXf yAttendLeft(tp/tEch) = Eigen::VectorXf::Constant(leftTrajectory(i, 1));

		Eigen::MatrixXf deplacementPied = AppendMatrixColumn(xAttendLeft, yAttendLeft);

		//Eigen::MatrixXf deplacementPied = AppendMatrix(mxbMatrix, tsn, 3);
		//Eigen::MatrixXf deplacementZMP = AppendMatrix(mxbMatrix, tsn, 3);
    }

	return trajectory;
}

/*
function DeplacementTemporel = ZMPtemporel( PA, Tp, Tech, ZMPlist)
%ZMPTEMPOREL Summary of this function goes here
%   Detailed explanation goes here

Tpn = 0:Tech:Tp;
% Temps 
Ts =0.2*Tp ;  %sec
Tsn = 0:Tech:Ts;

    TrajX = [];
    TrajY = [];

        Tempstotal = (length(ZMPlist)-1)*(Ts+Tp)
        
        tempDeplacement = [PA(1) PA(2) 0];
      
    for j = 1:1: length(ZMPlist)-1

        [DroiteX DroiteY ] = mxb(Tsn/Ts, ZMPlist(j,1:2), ZMPlist(j+1,1:2));
        DeplacementZMP = [DroiteX(2:end) ; DroiteY(2:end) ; Tsn(2:end)]';
        
        Xattend(1:Tp/Tech) = ZMPlist(j+1,1);
        Yattend(1:Tp/Tech) = ZMPlist(j+1,2);
        DeplacementPied = [ Xattend ; Yattend ; Tpn(2:end)]';
        
        Deplacement     = [DeplacementZMP ; DeplacementPied];
        tempDeplacement  = [tempDeplacement ; Deplacement];
  
    end
 
 Ttn = [0:Tech:Tempstotal]'  
DeplacementTemporel = [tempDeplacement(1:end,1), tempDeplacement(1:end,2), Ttn ];

end
*/



