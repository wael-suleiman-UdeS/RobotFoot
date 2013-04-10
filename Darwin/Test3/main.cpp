#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "LinuxDARwIn.h"


using namespace Robot;

int main()
{
	printf( "\n===== Read/Write Tutorial for DARwIn =====\n\n");

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730("/dev/ttyUSB0");
	CM730 cm730(&linux_cm730);
	if(cm730.Connect() == false)
	{
		printf("Fail to connect CM-730!\n");
		return 0;
	}
	/////////////////////////////////////////////////////////////////////

    //cm730.WriteByte(JointData::ID_R_ANKLE_ROLL, MX28::P_TORQUE_ENABLE, 1, 0);
    //cm730.WriteByte(JointData::ID_R_SHOULDER_ROLL,  MX28::P_TORQUE_ENABLE, 1, 0);
    //cm730.WriteByte(JointData::ID_R_ELBOW,          MX28::P_TORQUE_ENABLE, 1, 0);

/*
    cm730.WriteByte(JointData::ID_R_SHOULDER_PITCH, MX28::P_P_GAIN, 8, 0);
    cm730.WriteByte(JointData::ID_R_SHOULDER_ROLL,  MX28::P_P_GAIN, 8, 0);
    cm730.WriteByte(JointData::ID_R_ELBOW,          MX28::P_P_GAIN, 8, 0);
*/
    //cm730.WriteWord(JointData::ID_R_ANKLE_ROLL, MX28::P_GOAL_POSITION_L, MX28::Angle2Value(0), 0);
    int value;

    cm730.ReadWord(JointData::ID_R_KNEE, MX28::P_PRESENT_POSITION_L, &value, 0);
    //cm730.ReadWord(JointData::ID_R_KNEE, MX28::P_ALARM_SHUTDOWN, &value, 0);
    std::cout << value << std::endl;
	return 0;
}
