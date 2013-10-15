#include "Trajectory.h"
#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>

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

//*****************************Check if offset should be removed*************************************//
Eigen::MatrixXf Trajectory::MXB(Eigen::Vector2f pointA, Eigen::Vector2f pointB, float increment, int offset)
{   
    int matrixSize = (1/increment) - offset;
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
	//Trajectory from point A to left footprint
    Eigen::MatrixXf trajectory = MXB(pointA, leftTrajectory.row(0), increment);

    for(int i =0, j = 1; i < leftTrajectory.innerSize() - 1; ++i, ++j)
    {
		//Trajectory from left to right to left foot steps
		Eigen::MatrixXf mxbMatrix = CreateCombinedMXBMatrix(leftTrajectory, rightTrajectory, increment, i, j);

        Eigen::MatrixXf tempMatrix = AppendMatrixRow(trajectory, mxbMatrix);
        trajectory.swap(tempMatrix);
    }

    //Append the last step (left foot) to pointD
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


//************This method could be used only when there are many steps, otherwise it might be removed*******************************************//
//Create a combined mxb matrix for trajectories from A to B to A
Eigen::MatrixXf Trajectory::CreateCombinedMXBMatrix(Eigen::MatrixXf matrixA, Eigen::MatrixXf matrixB, float increment, int i, int j, int offset)
{
    //Create a matrix with zmp trajectory from A to B
    Eigen::MatrixXf mxbMatrixBA = MXB(matrixA.row(i), matrixB.row(j), increment, offset);
    
    //Append both step trajectories 
    Eigen::MatrixXf mxbMatrix;
    if(matrixA.rows() > i+1)
    {  
		//Create a matrix with zmp trajectory from B to A
        Eigen::MatrixXf mxbMatrixAB = MXB(matrixB.row(j), matrixA.row(i+1), increment, offset);
        mxbMatrix = AppendMatrixRow(mxbMatrixBA, mxbMatrixAB);  
    }
    else
    {
        mxbMatrix = mxbMatrixBA;
    }

	return mxbMatrix;
}

//Mettre pointeurs
Eigen::MatrixXf Trajectory::TemporalZMP(Eigen::MatrixXf zmpSteps, int tp, float tEch)
{
    float ts = 0.2*tp;

    //Create Tpn
    Eigen::VectorXf tpn = Eigen::VectorXf::LinSpaced(tp/tEch, 0, tp);
	
	//Initial position (X,Y,T)
    Eigen::Vector3f tempDeplacement(zmpSteps(0, 0), zmpSteps(0, 1), 0);

	Eigen::MatrixXf timeMVTMatrix;

	int attendVectorLength = tp/tEch;

    for(int i = 0; i < zmpSteps.innerSize() - 1; ++i)
    {
		Eigen::Vector2f firstStep = zmpSteps.row(i);
		Eigen::Vector2f secondStep = zmpSteps.row(i + 1);

		//Calculate trajectory between 2 footsteps
		Eigen::MatrixXf deplacementZMP = MXB(firstStep, secondStep, tEch/ts);

		Eigen::MatrixXf groundedFootZMP(attendVectorLength, 2);
		groundedFootZMP << Eigen::VectorXf::Constant(attendVectorLength, firstStep(0)), Eigen::VectorXf::Constant(attendVectorLength, firstStep(1));

		Eigen::MatrixXf deplacement = AppendMatrixRow(deplacementZMP, groundedFootZMP);

		Eigen::MatrixXf tempMatrix;
		if(timeMVTMatrix.innerSize() >0)
			tempMatrix = AppendMatrixRow(timeMVTMatrix, deplacement);
		else
			tempMatrix = deplacement;

		timeMVTMatrix.swap(tempMatrix);
	}

	//total time
	float totalTime = (zmpSteps.innerSize() - 1)*(ts + tp);

	//Create Ttn
    Eigen::VectorXf ttn = Eigen::VectorXf::LinSpaced(totalTime/tEch, 0, totalTime);

	Eigen::MatrixXf deplacementTemporel = AppendMatrixColumn(timeMVTMatrix, ttn);
	
	return deplacementTemporel;
}

void Trajectory::ParallelCurve(Eigen::VectorXf &xInner, Eigen::VectorXf &yInner, Eigen::VectorXf &xOuter, Eigen::VectorXf &yOuter, Eigen::VectorXf x, Eigen::VectorXf y, float d)
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

	Eigen::MatrixXf unv = AppendMatrixColumn(dy, -dx);

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

///////////////////////////////////////////////////////////////////////////////
//Does not belong in this class, im just being lazy
///////////////////////////////////////////////////////////////////////////////

//NOT TESTED
Eigen::MatrixXf Trajectory::UpdateDH(float L4, float L5, Eigen::VectorXf q)
{
	Eigen::MatrixXf DH = Eigen::MatrixXf::Zero(6,4);
	DH(0,1) = M_PI_2;
	DH(0,3) = q(1);
	DH(1,0) = L4;
	DH(1,3) = q(2);
	DH(2,0) = L5;
	DH(2,3) = q(3);
	DH(0,1) = -M_PI_2;
	DH(0,1) = q(4);
	DH(0,1) = M_PI_2;
	DH(0,1) = q(5)+M_PI_2;
	DH(0,1) = q(6);

	return DH;
}

//NOT TESTED
Eigen::Matrix4f Trajectory::MatrixHomogene(Eigen::MatrixXf DH)
{
	//find better names, MF & A
	Eigen::Matrix4f A = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f MF = Eigen::Matrix4f::Identity();

	for(int i = 0; i < DH.innerSize(); ++i)
	{
		A(0,0) = cos(DH(i,3)); A(0,1) = -sin(DH(i,3))*cos(DH(i,1)); A(0,2) = sin(DH(i,3))*sin(DH(i,1)); A(0,3) = DH(i,0)*cos(DH(i,3));
		A(0,0) = sin(DH(i,3)); A(0,1) = cos(DH(i,3))*cos(DH(i,1)); A(0,2) = -cos(DH(i,3))*sin(DH(i,1)); A(0,3) = DH(i,0)*sin(DH(i,3));
		A(0,0) = 0; 		   A(0,1) = sin(DH(i,1)); 				A(0,2) = cos(DH(i,1)); 				A(0,3) = DH(i,2);

		MF *= A;
	}

	return MF;
}

//NOT TESTED
Eigen::MatrixXf Trajectory::Jacobian(Eigen::MatrixXf DH, int returnChoice)
{
	Eigen::Matrix4f A01 = MatrixHomogene(DH.row(0));
	Eigen::Matrix4f A12 = MatrixHomogene(DH.row(1));
	Eigen::Matrix4f A23 = MatrixHomogene(DH.row(2));
	Eigen::Matrix4f A34 = MatrixHomogene(DH.row(3));
	Eigen::Matrix4f A45 = MatrixHomogene(DH.row(4));
	//Eigen::Matrix4f A56 = MatrixHomogene(DH.row(5)); //not used

	Eigen::Matrix4f A02 = A01*A12;
	Eigen::Matrix4f A03 = A02*A23;
	Eigen::Matrix4f A04 = A03*A34;
	Eigen::Matrix4f A05 = A04*A45;
	//Eigen::Matrix4f A06 = A05*A56; //not used

	Eigen::Vector4f Z1(0,0,1,0);
	Eigen::MatrixXf Z(4,6);
	Z << Z1, A01.col(2), A02.col(2), A03.col(2), A04.col(2), A05.col(2);
	Z.resize(3, Z.cols());

	Eigen::MatrixXf P(4,7);
	P << Eigen::Vector4f::Zero(), A01.col(3), A02.col(3), A03.col(3), A04.col(3), A05.col(3);
	P.resize(3, P.cols());

	Eigen::MatrixXf Jposition(3,6);
	Eigen::MatrixXf Jrotation = Z;//Clearer but Useless...

	//CROSS error: static assertion failed: "THIS_METHOD_IS_ONLY_FOR_VECTORS_OF_A_SPECIFIC_SIZE"
	//Eigen::VectorXf test = Z.zCol(0).cross(P.col(6)-P.col(0));

	//Correct, but lame
	//Eigen::Vector3f zCol = Z.col(0);
	//Eigen::Vector3f deltaP = P.col(6)-P.col(0);
	//Eigen::VectorXf test = zCol.cross(deltaP);

	//Make this work!
	//Jposition << Z.col(0).cross(P.col(6)-P.col(0)), Z.col(1).cross(P.col(6)-P.col(1)), Z.col(2).cross(P.col(6)-P.col(2)), Z.col(3).cross(P.col(6)-P.col(3), Z.col(4).cross(P.col(6)-P.col(4)), Z.col(5).cross(P.col(6)-P.col(5)));

	if(returnChoice > 2)
		return Jrotation;
	else if(returnChoice < 2)
	{
		Eigen::MatrixXf J(6,6);
		J << Jposition, Jrotation;
		return J;
	}
	else
		return Jposition;

}
