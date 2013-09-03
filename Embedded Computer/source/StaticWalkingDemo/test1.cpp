#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "../Control/STM32F4.h"

int main(int argc, char* argv[])
{
   try
   {
      boost::asio::io_service io; 
      STM32F4 mc(argc > 0 ? std::string("/dev/") + argv[0] : std::string("/dev/ttyUSB0"), io);
      boost::thread t(boost::bind(&boost::asio::io_service::run, &io));
      
      mc.setTorque(8,STM32F4::TorqueOn);
		
		std::cout << mc.read(8) << std::endl;
		std::string sPosition;
		getline(std::cin,sPosition);
		int iPosition = atoi(sPosition.c_str());
		mc.setMotor(8,iPosition);
		std::cout << "Before Sleep\n";
		sleep(2);
		std::cout << "After Sleep\n";
		getline(std::cin,sPosition);
		mc.setTorque(8,STM32F4::TorqueOff);
   }   
   catch (std::exception& e)
   {   
   std::cerr << "Exception :" << e.what() << std::endl;
   }   
	return 0;
}
