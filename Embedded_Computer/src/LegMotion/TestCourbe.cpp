/**
******************************************************************************^M
* @file    TestCourbe.cpp
* @authors  Camille Hébert
* @date    2013-12-4
* @brief   Utility class to test curved trajectories
******************************************************************************^M
*/

#include "TestCourbe.h"

#include <fstream>
#include <iostream>

using namespace std;

namespace TestCourbe
{
	Eigen::MatrixXf GetTrajectoriesFromFile(string filename)
	{
		Eigen::MatrixXf trajectories(1, 6);

		string line;

		std::ifstream ifs;
		ifs.open (filename.c_str(), std::ifstream::in);

		if (ifs.is_open())
		{
			int colCount = 0;
			int rowCount = 0;
			while(getline(ifs,line))
			{
				if( colCount > 5 )
				{
					colCount = 0;
					rowCount++;

					trajectories.conservativeResize(rowCount+1, 6);
				}

				trajectories(rowCount, colCount) = float(atof(line.c_str()));

				colCount++;
			}
			ifs.close();
		}
		else
		{
			printf("Cannot open the file.");
			throw;
		}

		return trajectories;

	}

	Eigen::VectorXf GetFixedFootFromFile(string filename)
	{
		Eigen::VectorXf fixedFoot(1);

		string line;

		std::ifstream ifs;
		ifs.open (filename.c_str(), std::ifstream::in);

		if (ifs.is_open())
		{
			int rowCount = 0;
			while(getline(ifs,line))
			{
				if(rowCount > 0)
					fixedFoot.conservativeResize(rowCount+1);

				fixedFoot(rowCount) = float(atof(line.c_str()));

				rowCount++;

			}
			ifs.close();
		}
		else
		{
			printf("Cannot open the file.");
			throw;
		}

		return fixedFoot;
	}
}

