/**
******************************************************************************^M
* @file    TestCourbe.h
* @authors  Camille Hébert
* @date    2013-12-4
* @brief   Utility class to test curved trajectories
******************************************************************************^M
*/

#ifndef TESTCOURBE_H
#define TESTCOURBE_H

#include "../../ThirdParty/Eigen/Dense"

namespace TestCourbe
{
	Eigen::MatrixXf GetTrajectoriesFromFile(std::string filename);
	Eigen::VectorXf GetFixedFootFromFile(std::string filename);
};

#endif  //TESTCOURBE_H
