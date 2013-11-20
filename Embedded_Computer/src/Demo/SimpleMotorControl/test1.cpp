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
void setPosition(STM32F4& mc, int id)
{
	std::string sPosition;

	std::cout << "Current position is : " << mc.read(id) << std::endl;

	std::cout << "Desired position : ";
	std::cin >> sPosition;
	mc.setMotor(id,atoi(sPosition.c_str()));
}

// Read motor position
void readPosition(STM32F4& mc, int id)
{
	std::string sPosition;

	std::cout << "Current position is : " << mc.read(id) << std::endl;
}

// Read motor status
void readStatus(STM32F4& mc, int id)
{
	std::string sPosition;

	std::cout << "Status is : " << mc.readStatus(id) << std::endl;
}

// Clear motor status
void clearStatus(STM32F4& mc, int id)
{
	std::string sPosition;

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

// Write on RAM adress
void writeRAM(STM32F4& mc, int id)
{
	int adress, value;
	std::string sPosition;

    std::cout << "Adress to write : ";
    std::cin >> adress; 
    
    std::cout << "Value to write : ";
    std::cin >> value; 

    mc.writeRAM(id,adress,value);
}

// Read a RAM adress
void readRAM(STM32F4& mc, int id)
{
	int adress;
	std::string sPosition;

    std::cout << "Adress to read : ";
    std::cin >> adress; 

    int value = mc.readRAM(id,adress);
	std::cout << "Value is : " << value << "  " << std::hex << value << std::dec << std::endl;
}

// Write value on adress for all motor
void writeAllRAM(STM32F4& mc)
{
	int adress, value;
	std::string sPosition;

    std::cout << "Adress to write : ";
    std::cin >> adress; 
    
    std::cout << "Value to write : ";
    std::cin >> value; 

	for(int id = 1; id < 15; id++)
	{
		mc.writeRAM(id,adress,value);
	}
	mc.writeRAM(253,adress,value);
}

void setAllTorque(STM32F4& mc)
{
	for(int id = 1; id < 15; id++)
	{
		mc.setTorque(id,STM32F4::TorqueOn);
	}
	mc.setTorque(253,STM32F4::TorqueOn);
}

void setOffAllTorque(STM32F4& mc)
{
	for(int id = 1; id < 15; id++)
	{
		mc.setTorque(id,STM32F4::TorqueOff);
	}
	mc.setTorque(253,STM32F4::TorqueOff);
}

int main(int argc, char* argv[])
{
   try
   {
      boost::asio::io_service io; 
      STM32F4 mc(argc > 1 ? std::string("/dev/") + argv[1] : std::string("/dev/ttyACM0"), io);
      boost::thread t(boost::bind(&boost::asio::io_service::run, &io));
      
      int id;
      std::cout << "Choose ID : ";
      std::cin >> id ; 
		while(1)
		{
			std::cout << "Choose Command (Set Position = 1, Read Position = 2, Read Status = 3, Clear Status = 4, Clear All Status = 5, Write RAM = 6, Read RAM = 7, Write All RAM = 8, Set All Torque = 9, Set Off Torque = 10,  End = 0)" << std::endl;
			int choice;
			std::cin >> choice;
			switch(choice)
			{
				case 0:
					return 0;
					break;
				case 1:
					setPosition(mc, id);
					break;
                case 2:
                    readPosition(mc, id);
                    break;
                case 3:
					readStatus(mc, id);
					break;
				case 4:
					clearStatus(mc, id);
					break;
				case 5:
					clearAllStatus(mc);
					break;
				case 6:
					writeRAM(mc, id);
					break;
				case 7:
					readRAM(mc, id);
					break;
                case 8:
                    writeAllRAM(mc);
                    break;
                case 9:
                    setAllTorque(mc);
                    break;
                case 10:
                    setOffAllTorque(mc);
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
