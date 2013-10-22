#include "WalkStatus.h"
#include "StaticWalk.h"

#include "../../Utilities/logger.h"
#include "../../Control/STM32F4.h"
#include "../../Control/MotorControl.h"

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <sstream>
#include <iterator>

const double uDt = 0.016; // Time step in second
const int msInitializationTime = 7000;

int main(int argc, char* argv[])
{   
	//TODO Add argument for activate (RightLeg,LeftLeg,AllLeg) and check if file and calcul fit this..
	//TODO Add argument for number of loop...
	if(argc > 1)
	{
		std::string filename;
		bool isUsingAlgorithm(false), isMotorActivated(false);

      const std::string help = "-help";
		const std::string usingAlgorithm = "-ua";
		const std::string motorActivated = "-am";
		const std::string debug = "-de";
 
		if(argv[1] == help)
		{
			std::cout << "First argument : File name of text file containing motor position. (Always needed for the initial position)" << std::endl;
			std::cout << "-ua : Use your algorithm." << std::endl
						 << "-am : Activate Motor." << std::endl
						 << "-de : Show debug message." << std::endl;
			return 0;
		}	
		else
		{
			filename = argv[1];
		}

		for(int i = 2; i < argc; i++)
		{
			if(argv[i] == usingAlgorithm)
			{
				isUsingAlgorithm = true;
			}
			else if(argv[i] == motorActivated)
			{
				isMotorActivated = true;
			}
			else if(argv[i] == debug)
			{
				Logger::getInstance().addStream(std::cout);
			}
			else
			{
				std::cout << "Invalid argument" << std::endl;
				return 1;
			}
		}

        boost::asio::io_service io; 
    	STM32F4 mc(std::string("/dev/ttyACM0"), io);
    	boost::thread t(boost::bind(&boost::asio::io_service::run, &io));

        StaticWalk staticWalk(&mc);
        staticWalk.init(filename, isUsingAlgorithm, isMotorActivated);
        staticWalk.initPosition(msInitializationTime);
        staticWalk.run(uDt);
	}
	else
	{
		std::cout << "No argument" << std::endl;
		return 1;
	}

  return 0;
}


