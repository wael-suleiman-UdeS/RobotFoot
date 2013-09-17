#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "STM32F4.h"
#include <vector>

class MotorControl
{
public:
   enum Option {
      ALL_LEGS = 0,
      RIGHT_LEGS = 1,
      LEFT_LEGS = 2,
      NUM_TEST
   };

   MotorControl(STM32F4* stm32f4);
   ~MotorControl();

   bool SetTorque( const Option option );
   //bool DisableTorque( const Option option );
  
   bool InitPosition( const std::vector<double>& vPos,
		       const Option option,
		       const double msTotalTime = 10000.0,
		       const double msDt = 16 );	
   
   bool SetPosition( const std::vector<double>& pos, const Option option );
   bool ReadPosition( std::vector<double>& pos, const Option option );
   

   
   

   struct Motor
   {
      int id;
      int offset;
      int minLimit;
      int maxLimit;
   };
   
	int Angle2Value(const int ID, const double angle);

    Motor R_HIP_YAW;
    Motor L_HIP_YAW;
    Motor R_HIP_ROLL;
    Motor L_HIP_ROLL;
    Motor R_HIP_PITCH;
    Motor L_HIP_PITCH;
    Motor R_KNEE;
    Motor L_KNEE;
    Motor R_ANKLE_PITCH;
    Motor L_ANKLE_PITCH;
    Motor R_ANKLE_ROLL;
    Motor L_ANKLE_ROLL;
   
    std::vector<Motor> motors;

private:

   struct Configuration
   {
      std::vector<int> joints;
      //int P;
      //int I;
      //int D;
   };



   void InitializeMotors();

   unsigned int getJointNum( const Option option );

	STM32F4* _stm32f4;
    Configuration _config[NUM_TEST];



	int Angle2Value(const double angle);

	double Value2Angle(const int value);
	double Value2Angle(const int ID, const int value);
};

#endif  //MOTOR_CONTROL_H
