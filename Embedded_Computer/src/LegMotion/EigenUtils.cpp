/**
******************************************************************************^M
* @file    EigenUtils.cpp
* @authors  Camille Hébert & Antoine Rioux
* @date    2013-11-19
* @brief   Utility class to calculate some trajectories
******************************************************************************^M
*/


#include "EigenUtils.h"

namespace EigenUtils
{
	const float dampingPinv = 0.1f;

	Eigen::MatrixXf MXB(Eigen::Vector2f pointA, Eigen::Vector2f pointB, float increment, int offset)
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

	//************This method could be used only when there are many steps, otherwise it might be removed*******************************************//
	//Create a combined mxb matrix for trajectories from A to B to A
	Eigen::MatrixXf CreateCombinedMXBMatrix(Eigen::MatrixXf matrixA, Eigen::MatrixXf matrixB, float increment, int i, int j, int offset)
	{
	    //Create a matrix with zmp trajectory from A to B
	    Eigen::MatrixXf mxbMatrixBA = MXB(matrixA.row(i), matrixB.row(j), increment, offset);

		Eigen::MatrixXf noZMPMovement(mxbMatrixBA.rows(), mxbMatrixBA.cols());
		for(int i = 0; i < mxbMatrixBA.rows(); i++)
		{
			noZMPMovement.row(i) = mxbMatrixBA.bottomRows(1);
		}

        Eigen::MatrixXf tempMatrix(2*mxbMatrixBA.rows(), mxbMatrixBA.cols());
        tempMatrix << mxbMatrixBA, noZMPMovement;
        mxbMatrixBA.swap(tempMatrix);

	    //Append both step trajectories
	    if(matrixA.rows() > i+1)
	    {
			//Create a matrix with zmp trajectory from B to A
	        Eigen::MatrixXf mxbMatrixAB = MXB(matrixB.row(j), matrixA.row(i+1), increment, offset);

			for(int i = 0; i < mxbMatrixAB.rows(); i++)
			{
				noZMPMovement.row(i) = mxbMatrixAB.bottomRows(1);
			}

	        Eigen::MatrixXf tempMatrix2(2*mxbMatrixAB.rows(), mxbMatrixAB.cols());
	        tempMatrix2 << mxbMatrixAB, noZMPMovement;
	        mxbMatrixAB.swap(tempMatrix2);

	        Eigen::MatrixXf mxbMatrix(mxbMatrixBA.rows()+mxbMatrixAB.rows(), mxbMatrixBA.cols());
	        mxbMatrix << mxbMatrixBA, mxbMatrixAB;

	        return mxbMatrix;
	    }
	    else
	    {
	        return mxbMatrixBA;
	    }
	}

	Eigen::MatrixXf PseudoInverse(Eigen::MatrixXf matrix)
	{
		Eigen::Matrix3f squaredMatrix = matrix*(matrix.transpose());
		Eigen::Matrix3f dampedIdentity = pow(dampingPinv,2)*Eigen::Matrix3f::Identity();
		return matrix.transpose()*((squaredMatrix+dampedIdentity).inverse());// NOT VERFIED, MAY CAUSE BUGS!
	}

	Eigen::Matrix3f BaseChange(float angle)
	{
		Eigen::Matrix3f matrix;
		matrix << cos(angle), -sin(angle), 0.0f, sin(angle), cos(angle), 0.0f, 0.0f, 0.0f, 1.0f;
		return matrix;
	}

}
