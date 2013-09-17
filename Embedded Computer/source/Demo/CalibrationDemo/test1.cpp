#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/thread.hpp>


#include "../../Control/STM32F4.h"
#include "../../Control/MotorControl.h"

int main(int argc, char* argv[])
{
                boost::asio::io_service io;
                STM32F4 mc(argc > 1 ? std::string("/dev/") + argv[1] : std::string("/dev/ttyUSB0"), io);
                boost::thread t(boost::bind(&boost::asio::io_service::run, &io));

                //Test calib motors
                MotorControl* motorsCalib = new MotorControl(&mc);
                motorsCalib->Angle2Value(motorsCalib->L_ANKLE_PITCH.id, 180);
                motorsCalib->Angle2Value(motorsCalib->L_ANKLE_PITCH.id, 0);
                motorsCalib->Angle2Value(motorsCalib->L_ANKLE_PITCH.id, 360);
  
	return 0;
}
