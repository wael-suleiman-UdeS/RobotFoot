#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "CM730.h"

#include <vector>

using namespace Robot;

class MotorControl
{
public:
   enum Option {
      ALL_LEGS = 0,
      RIGHT_LEGS = 1,
      LEFT_LEGS = 2
      NUM_TEST
   };

   MotorControl( CM730* cm730 );
   ~MotorControl();

   bool SetTorque( const Option option );
   bool DisableTorque( const Option option );
  
   bool InitPosition( const std::vector<double>& vPos,
		       const Option option,
		       const double msTotalTime = 10000.0,
		       const double msDt = 16 );	
   
   bool SetPosition( const std::vector<double>& pos, const Option option );
   bool ReadPosition( std::vector<double>& pos, const Option option );
   
private:
   
   struct Configuration
   {
      std::vector<int> joints;
      int P;
      int I;
      int D;    
   };
   
   unsigned int getJointNum( const Option option );
   
   CM730* _cm730;
   Configuration _config[NUM_TEST];
   
};

#endif  //MOTOR_CONTROL_H
