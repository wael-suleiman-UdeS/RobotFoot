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

private:

   struct Motor
   {
      int id;
      int offset;
      int minLimit;
      int maxLimit;
   };   

   struct Configuration
   {
      std::vector<Motor> joints;
   };

   void InitializeMotors(std::map<std::string, Motor>&);

   unsigned int getJointNum( const Option option );

    STM32F4* _stm32f4;
    Configuration _config[NUM_TEST];


    int Angle2Value(const double angle);
    int Angle2Value(const Motor& motor, const double angle);
    double Value2Angle(const int value);
    double Value2Angle(const Motor& motor, const int value);
};

#endif  //MOTOR_CONTROL_H
