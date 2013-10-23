#ifndef EIGENUTILS_H
#define EIGENUTILS_H

#include "../../ThirdParty/Eigen/Dense"

namespace EigenUtils
{

	Eigen::MatrixXf MXB(Eigen::Vector2f pointA, Eigen::Vector2f pointB, float increment, int offset = 0);
	Eigen::MatrixXf CreateCombinedMXBMatrix(Eigen::MatrixXf matrixA, Eigen::MatrixXf matrixB, float increment, int i, int j, int offset = 0);

};

#endif  //EIGENUTILS_H
