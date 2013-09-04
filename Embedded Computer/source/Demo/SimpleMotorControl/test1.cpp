#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "../../Control/STM32F4.h"

int main(int argc, char* argv[])
{
   try
   {
      boost::asio::io_service io; 
      STM32F4 mc(argc > 1 ? std::string("/dev/") + argv[1] : std::string("/dev/ttyUSB0"), io);
      boost::thread t(boost::bind(&boost::asio::io_service::run, &io));
      
		while(1)
		{
			int id;
			std::string sPosition;

			std::cout << "Enter motor id : ";
			std::cin >> id;

			if(id==0)
				return 0;

			std::cout << "Current position is : " << mc.read(id) << std::endl;

			mc.setTorque(id,STM32F4::TorqueOn);
			std::cout << "Desired position : ";
			std::cin >> sPosition;
			mc.setMotor(id,atoi(sPosition.c_str()));
			sleep(2);
			mc.setTorque(id,STM32F4::TorqueOff);

			std::cout << std::endl;
 		}
      
   }   
   catch (std::exception& e)
   {   
   std::cerr << "Exception :" << e.what() << std::endl;
   }   
	return 0;
}
