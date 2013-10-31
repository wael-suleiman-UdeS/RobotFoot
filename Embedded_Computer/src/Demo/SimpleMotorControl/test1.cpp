/**
******************************************************************************^M
* @file    test1.cpp
* @author  Mathieu Drapeau
* @date    2013-09-20
* @brief   Quick tools to communicate with motors
******************************************************************************^M
*/
#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "../../Control/STM32F4.h"

// Read current position and set a new position
void setPosition(STM32F4& mc)
{
	int id;
	std::string sPosition;

	std::cout << "Enter motor id : ";
	std::cin >> id;
	std::cout << "Current position is : " << mc.read(id) << std::endl;

	mc.setTorque(id,STM32F4::TorqueOn);
	std::cout << "Desired position : ";
	std::cin >> sPosition;
	mc.setMotor(id,atoi(sPosition.c_str()));
	sleep(2);
	mc.setTorque(id,STM32F4::TorqueOff);
}

// Read motor status
void readStatus(STM32F4& mc)
{
	int id;
	std::string sPosition;

	std::cout << "Enter motor id : ";
	std::cin >> id;

	std::cout << "Status is : " << mc.readStatus(id) << std::endl;
}

// Clear motor status
void clearStatus(STM32F4& mc)
{
	int id;
	std::string sPosition;

	std::cout << "Enter motor id : ";
	std::cin >> id;

	mc.clearStatus(id);
}

// Clear all motor status
void clearAllStatus(STM32F4& mc)
{
	for(int id = 1; id < 15; id++)
	{
		mc.clearStatus(id);
	}
	mc.clearStatus(253);
}

int main(int argc, char* argv[])
{
   try
   {
      boost::asio::io_service io; 
      STM32F4 mc(argc > 1 ? std::string("/dev/") + argv[1] : std::string("/dev/ttyACM0"), io);
      boost::thread t(boost::bind(&boost::asio::io_service::run, &io));
      
		while(1)
		{
			std::cout << "Choose Command (Set Position = 1, Read Status = 2, Clear Status = 3, Clear All Status = 4, End = 0)" << std::endl;
			int choice;
			std::cin >> choice;
			switch(choice)
			{
				case 0:
					return 0;
					break;
				case 1:
					setPosition(mc);
					break;
				case 2:
					readStatus(mc);
					break;
				case 3:
					clearStatus(mc);
					break;
				case 4:
					clearAllStatus(mc);
					break;
			}
 
			std::cout << std::endl;
 		}
      
   }   
   catch (std::exception& e)
   {   
   std::cerr << "Exception :" << e.what() << std::endl;
   }   
	return 0;
}
