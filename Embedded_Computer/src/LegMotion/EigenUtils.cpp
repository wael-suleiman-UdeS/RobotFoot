#include "EigenUtils.h"

namespace EigenUtils
{

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

	Eigen::MatrixXf AppendMatrixRow(Eigen::MatrixXf matrixA, Eigen::MatrixXf matrixB)
	{
	    Eigen::MatrixXf appendedMatrix(matrixA.rows()+matrixB.rows(), matrixA.cols());
	    appendedMatrix << matrixA, matrixB;

	    return appendedMatrix;
	}

	Eigen::MatrixXf AppendMatrixColumn(Eigen::MatrixXf matrixA, Eigen::MatrixXf matrixB)
	{
	    Eigen::MatrixXf appendedMatrix(matrixA.rows(), matrixA.cols()+matrixB.cols());
	    appendedMatrix << matrixA, matrixB;

	    return appendedMatrix;
	}

	//************This method could be used only when there are many steps, otherwise it might be removed*******************************************//
	//Create a combined mxb matrix for trajectories from A to B to A
	Eigen::MatrixXf CreateCombinedMXBMatrix(Eigen::MatrixXf matrixA, Eigen::MatrixXf matrixB, float increment, int i, int j, int offset)
	{
	    //Create a matrix with zmp trajectory from A to B
	    Eigen::MatrixXf mxbMatrixBA = MXB(matrixA.row(i), matrixB.row(j), increment, offset);

	    //Append both step trajectories
	    if(matrixA.rows() > i+1)
	    {
			//Create a matrix with zmp trajectory from B to A
	        Eigen::MatrixXf mxbMatrixAB = MXB(matrixB.row(j), matrixA.row(i+1), increment, offset);

	        Eigen::MatrixXf mxbMatrix(mxbMatrixBA.rows()+mxbMatrixAB.rows(), mxbMatrixBA.cols());
	        mxbMatrix << mxbMatrixBA, mxbMatrixAB;

	        return mxbMatrix;
	    }
	    else
	    {
	        return mxbMatrixBA;
	    }
	}

}
