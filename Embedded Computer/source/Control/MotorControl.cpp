#include "MotorControl.h"

#include <iostream>
#include <iterator>
#include <algorithm>
#include <functional>

//#define DEBUG_TEST_MOTION
#define DANGER_TEST_MOTION

namespace
{
	//Right Leg
	const int ID_R_HIP_YAW = 1;
	const int ID_R_HIP_ROLL = 3;
	//TODO should be 5 temp test
	const int ID_R_HIP_PITCH = 253;
	const int ID_R_KNEE = 7;
	const int ID_R_ANKLE_PITCH = 9;
	const int ID_R_ANKLE_ROLL = 11;
	//Left leg
	const int ID_L_HIP_YAW = 2;
	const int ID_L_HIP_ROLL = 4;
	const int ID_L_HIP_PITCH = 6;
	const int ID_L_KNEE = 8;
	const int ID_L_ANKLE_PITCH = 10;
	const int ID_L_ANKLE_ROLL = 12;

	const double dAngleConvertion = 0.325;
	const double dInvAngleConvertion = 1/dAngleConvertion;
	const int iOffsetValue = 512;
}

MotorControl::MotorControl( STM32F4* stm32f4 ) :
 _stm32f4(stm32f4)
{  
   std::vector<int> joints;
   //Right Leg
   joints.push_back(ID_R_HIP_YAW);
   joints.push_back(ID_R_HIP_ROLL);
   joints.push_back(ID_R_HIP_PITCH);
   joints.push_back(ID_R_KNEE);
   joints.push_back(ID_R_ANKLE_PITCH);
   joints.push_back(ID_R_ANKLE_ROLL);
   //Left leg
   joints.push_back(ID_L_HIP_YAW);
   joints.push_back(ID_L_HIP_ROLL);
   joints.push_back(ID_L_HIP_PITCH);
   joints.push_back(ID_L_KNEE);
   joints.push_back(ID_L_ANKLE_PITCH);
   joints.push_back(ID_L_ANKLE_ROLL);
   
   _config[ALL_LEGS].joints = joints;
   //_config[ALL_LEGS].P = JointData::P_GAIN_DEFAULT;
   //_config[ALL_LEGS].I = JointData::I_GAIN_DEFAULT;
   //_config[ALL_LEGS].D =JointData::D_GAIN_DEFAULT;  
      
   joints.clear();
   //Right Leg
   joints.push_back(ID_R_HIP_YAW);
   joints.push_back(ID_R_HIP_ROLL);
   joints.push_back(ID_R_HIP_PITCH);
   joints.push_back(ID_R_KNEE);
   joints.push_back(ID_R_ANKLE_PITCH);
   joints.push_back(ID_R_ANKLE_ROLL);
   
   _config[RIGHT_LEGS].joints = joints;
   //_config[RIGHT_LEGS].P = JointData::P_GAIN_DEFAULT;
   //_config[RIGHT_LEGS].I = JointData::I_GAIN_DEFAULT;
   //_config[RIGHT_LEGS].D =JointData::D_GAIN_DEFAULT;
   
   joints.clear();
   //Left leg
   joints.push_back(ID_L_HIP_YAW);
   joints.push_back(ID_L_HIP_ROLL);
   joints.push_back(ID_L_HIP_PITCH);
   joints.push_back(ID_L_KNEE);
   joints.push_back(ID_L_ANKLE_PITCH);
   joints.push_back(ID_L_ANKLE_ROLL);
   
   _config[LEFT_LEGS].joints = joints;
   //_config[LEFT_LEGS].P = JointData::P_GAIN_DEFAULT;
   //_config[LEFT_LEGS].I = JointData::I_GAIN_DEFAULT;
   //_config[LEFT_LEGS].D =JointData::D_GAIN_DEFAULT; 
   
}

MotorControl::~MotorControl()
{

}

bool MotorControl::SetTorque( const Option option )
{
   bool status = true;
   
   std::vector<int>::const_iterator itr = _config[ option ].joints.begin();
   const std::vector<int>::const_iterator end = _config[ option ].joints.end();
   
   for( ; itr != end && status ; itr++ )
   {
     // TODO : Grab status
      //status &=
		_stm32f4->setTorque(*itr,STM32F4::TorqueOn);
      //_cm730->WriteByte(*itr, MX28::P_P_GAIN, JointData::P_GAIN_DEFAULT, 0);
      //_cm730->WriteByte(*itr, MX28::P_I_GAIN, JointData::I_GAIN_DEFAULT, 0);
      //_cm730->WriteByte(*itr, MX28::P_D_GAIN, JointData::D_GAIN_DEFAULT, 0);
		//TODO : temporary test 
		//sleep(1);
   }
   
   return status;
}
/*
bool MotorControl::DisableTorque( const Option option )
{
   bool status = true;
   
   std::vector<int>::const_iterator itr = _config[ option ].joints.begin();
   const std::vector<int>::const_iterator end = _config[ option ].joints.end();
   
   for( ; itr != end && status ; itr++ )
   {
     // TODO : Grab status
      //status &= 
		_cm730->WriteByte( *itr, MX28::P_TORQUE_ENABLE, 0, 0);
   }
   
   return status;
}
*/
bool MotorControl::InitPosition( const std::vector<double>& desiredPos,
				  const Option option,
				  const double msTotalTime /*= 10000.0*/,
				  const double msDt /*= 16*/ )
{
   if( desiredPos.size() != getJointNum( option ) )
   {
#ifdef DEBUG_TEST_MOTION
      std::cout << "In function \"InitPosition\" : Joint number is invalid."
                << std::endl;
#endif
      return false;
   }
   
#ifdef DEBUG_TEST_MOTION
   std::cout << "Setting initial position : \n";
   std::copy(desiredPos.begin(), desiredPos.end(), std::ostream_iterator<double>(std::cout, " "));
#endif   
      
   std::vector<double> pos;
   if( !ReadPosition( pos, option ) )
   {
#ifdef DEBUG_TEST_MOTION      
      std::cout << "Error reading motor position";
#endif
      return false;
   }
   
   std::vector<double> posDt;
   
   std::vector<double>::const_iterator itrDesiredPos = desiredPos.begin();
   const std::vector<double>::const_iterator endDesiredPos = desiredPos.end();
   std::vector<double>::iterator itrPos = pos.begin();
   const std::vector<double>::iterator endPos = pos.end();
   
   // Calcul position dt for each motor
   for( ; itrDesiredPos != endDesiredPos || itrPos != endPos; itrDesiredPos++, itrPos++ )
   {
      posDt.push_back( (*itrDesiredPos - *itrPos)/(msTotalTime/msDt) );
   }
   
   // Loop to add position dt to motor position and send command to motor
   for(double t = 0.0; t < msTotalTime; t+=msDt)
   {
      std::transform(pos.begin(), pos.end(), posDt.begin(), pos.begin(), std::plus<double>());
      
      SetPosition( pos, option );
      
      usleep(msDt*1000);
   }
   
   return true;
}

bool MotorControl::SetPosition( const std::vector<double>& pos, const Option option )
{
   if( pos.size() != getJointNum( option ) )
   {
#ifdef DEBUG_TEST_MOTION
      std::cout << "In function \"SetPosition\" : Joint number is invalid."
                << std::endl;
#endif
      return false;
   }
  
   bool status = true;
   
#ifdef DANGER_TEST_MOTION   
   std::vector<int>::const_iterator itrJoint = _config[ option ].joints.begin();
   const std::vector<int>::const_iterator endJoint = _config[ option ].joints.end();
   std::vector<double>::const_iterator itrPos = pos.begin();
   const std::vector<double>::const_iterator endPos = pos.end();
   
   for( ; itrJoint != endJoint && itrPos != endPos && status ; itrJoint++, itrPos++ )
   {
		_stm32f4->setMotor(*itrJoint,Angle2Value(*itrPos));		      
   }
#endif

#ifdef DEBUG_TEST_MOTION
      std::copy(pos.begin(), pos.end(), std::ostream_iterator<double>(std::cout, " "));
      std::cout << std::endl;
#endif
   
   return status;
}

bool MotorControl::ReadPosition( std::vector<double>& pos, const Option option )
{
   bool status = true;
   int value;
   
   std::vector<int>::const_iterator itr = _config[ option ].joints.begin();
   const std::vector<int>::const_iterator end = _config[ option ].joints.end();
   
   for( ; itr != end && status ; itr++ )
   {
		value = _stm32f4->read(*itr);      
      pos.push_back( Value2Angle(value) );
   }
   
   return status;
}

unsigned int MotorControl::getJointNum( const Option option )
{
   return _config[ option ].joints.size();
}

int MotorControl::Angle2Value(const double angle)
{
	return (angle*dInvAngleConvertion)+iOffsetValue;
}

double MotorControl::Value2Angle(const int value)
{
	return double(value-iOffsetValue)*dAngleConvertion;
}
