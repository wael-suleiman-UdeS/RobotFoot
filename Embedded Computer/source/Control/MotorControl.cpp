#include "MotorControl.h"
#include "../Utilities/XmlParser.h"
#include "../Utilities/logger.h"

#include <iostream>
#include <iterator>
#include <algorithm>
#include <functional>
#include <boost/algorithm/clamp.hpp>

#define DEBUG_TEST_MOTION

using boost::algorithm::clamp;


#define DANGER_TEST_MOTION

namespace
{
	const double dAngleConvertion = 0.325;
	const double dInvAngleConvertion = 1/dAngleConvertion;
}

MotorControl::MotorControl( STM32F4* stm32f4 ) :
 _stm32f4(stm32f4)
{
	InitializeMotors();

   std::vector<int> joints;
   //Right Leg
   joints.push_back(R_HIP_YAW.id);
   joints.push_back(R_HIP_ROLL.id);
   joints.push_back(R_HIP_PITCH.id);
   joints.push_back(R_KNEE.id);
   joints.push_back(R_ANKLE_PITCH.id);
   joints.push_back(R_ANKLE_ROLL.id);
   //Left leg
   joints.push_back(L_HIP_YAW.id);
   joints.push_back(L_HIP_ROLL.id);
   joints.push_back(L_HIP_PITCH.id);
   joints.push_back(L_KNEE.id);
   joints.push_back(L_ANKLE_PITCH.id);
   joints.push_back(L_ANKLE_ROLL.id);

   _config[ALL_LEGS].joints = joints;
   //_config[ALL_LEGS].P = JointData::P_GAIN_DEFAULT;
   //_config[ALL_LEGS].I = JointData::I_GAIN_DEFAULT;
   //_config[ALL_LEGS].D =JointData::D_GAIN_DEFAULT;

   joints.clear();
   //Right Leg
   joints.push_back(R_HIP_YAW.id);
   joints.push_back(R_HIP_ROLL.id);
   joints.push_back(R_HIP_PITCH.id);
   joints.push_back(R_KNEE.id);
   joints.push_back(R_ANKLE_PITCH.id);
   joints.push_back(R_ANKLE_ROLL.id);

   _config[RIGHT_LEGS].joints = joints;
   //_config[RIGHT_LEGS].P = JointData::P_GAIN_DEFAULT;
   //_config[RIGHT_LEGS].I = JointData::I_GAIN_DEFAULT;
   //_config[RIGHT_LEGS].D =JointData::D_GAIN_DEFAULT;

   joints.clear();
   //Left leg
   joints.push_back(L_HIP_YAW.id);
   joints.push_back(L_HIP_ROLL.id);
   joints.push_back(L_HIP_PITCH.id);
   joints.push_back(L_KNEE.id);
   joints.push_back(L_ANKLE_PITCH.id);
   joints.push_back(L_ANKLE_ROLL.id);

   _config[LEFT_LEGS].joints = joints;
   //_config[LEFT_LEGS].P = JointData::P_GAIN_DEFAULT;
   //_config[LEFT_LEGS].I = JointData::I_GAIN_DEFAULT;
   //_config[LEFT_LEGS].D =JointData::D_GAIN_DEFAULT;

}

MotorControl::~MotorControl()
{

}

void MotorControl::InitializeMotors()
{
	XmlParser config;
	if (config.loadFile("config.xml"))
	{
		R_HIP_YAW.id  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_HIP_YAW / XmlPath::MotorID);
		R_HIP_YAW.offset  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_HIP_YAW / XmlPath::Offset);
		R_HIP_YAW.minLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_HIP_YAW / XmlPath::LimitMin);
		R_HIP_YAW.maxLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_HIP_YAW / XmlPath::LimitMax);

		L_HIP_YAW.id  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_HIP_YAW / XmlPath::MotorID);
		L_HIP_YAW.offset  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_HIP_YAW / XmlPath::Offset);
		L_HIP_YAW.minLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_HIP_YAW / XmlPath::LimitMin);
		L_HIP_YAW.maxLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_HIP_YAW / XmlPath::LimitMax);

		R_HIP_ROLL.id  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_HIP_ROLL / XmlPath::MotorID);
		R_HIP_ROLL.offset  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_HIP_ROLL / XmlPath::Offset);
		R_HIP_ROLL.minLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_HIP_ROLL / XmlPath::LimitMin);
		R_HIP_ROLL.maxLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_HIP_ROLL / XmlPath::LimitMax);

		L_HIP_ROLL.id  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_HIP_ROLL / XmlPath::MotorID);
		L_HIP_ROLL.offset  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_HIP_ROLL / XmlPath::Offset);
		L_HIP_ROLL.minLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_HIP_ROLL / XmlPath::LimitMin);
		L_HIP_ROLL.maxLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_HIP_ROLL / XmlPath::LimitMax);

		R_HIP_PITCH.id  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_HIP_PITCH / XmlPath::MotorID);
		R_HIP_PITCH.offset  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_HIP_PITCH / XmlPath::Offset);
		R_HIP_PITCH.minLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_HIP_PITCH / XmlPath::LimitMin);
		R_HIP_PITCH.maxLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_HIP_PITCH / XmlPath::LimitMax);

		L_HIP_PITCH.id  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_HIP_PITCH / XmlPath::MotorID);
		L_HIP_PITCH.offset  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_HIP_PITCH / XmlPath::Offset);
		L_HIP_PITCH.minLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_HIP_PITCH / XmlPath::LimitMin);
		L_HIP_PITCH.maxLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_HIP_PITCH / XmlPath::LimitMax);

		R_KNEE.id  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_KNEE / XmlPath::MotorID);
		R_KNEE.offset  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_KNEE / XmlPath::Offset);
		R_KNEE.minLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_KNEE / XmlPath::LimitMin);
		R_KNEE.maxLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_KNEE / XmlPath::LimitMax);

		L_KNEE.id  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_KNEE / XmlPath::MotorID);
		L_KNEE.offset  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_KNEE / XmlPath::Offset);
		L_KNEE.minLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_KNEE / XmlPath::LimitMin);
		L_KNEE.maxLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_KNEE / XmlPath::LimitMax);

		R_ANKLE_PITCH.id  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_ANKLE_PITCH / XmlPath::MotorID);
		R_ANKLE_PITCH.offset  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_ANKLE_PITCH / XmlPath::Offset);
		R_ANKLE_PITCH.minLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_ANKLE_PITCH / XmlPath::LimitMin);
		R_ANKLE_PITCH.maxLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_ANKLE_PITCH / XmlPath::LimitMax);

		L_ANKLE_PITCH.id  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_ANKLE_PITCH / XmlPath::MotorID);
		L_ANKLE_PITCH.offset  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_ANKLE_PITCH / XmlPath::Offset);
		L_ANKLE_PITCH.minLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_ANKLE_PITCH / XmlPath::LimitMin);
		L_ANKLE_PITCH.maxLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_ANKLE_PITCH / XmlPath::LimitMax);

		R_ANKLE_ROLL.id  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_ANKLE_ROLL / XmlPath::MotorID);
		R_ANKLE_ROLL.offset  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_ANKLE_ROLL / XmlPath::Offset);
		R_ANKLE_ROLL.minLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_ANKLE_ROLL / XmlPath::LimitMin);
		R_ANKLE_ROLL.maxLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::R_ANKLE_ROLL / XmlPath::LimitMax);

		L_ANKLE_ROLL.id  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_ANKLE_ROLL / XmlPath::MotorID);
		L_ANKLE_ROLL.offset  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_ANKLE_ROLL / XmlPath::Offset);
		L_ANKLE_ROLL.minLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_ANKLE_ROLL / XmlPath::LimitMin);
		L_ANKLE_ROLL.maxLimit  = config.getIntValue(XmlPath::LegsMotors / XmlPath::L_ANKLE_ROLL / XmlPath::LimitMax);

		motors.push_back(R_HIP_YAW);
		motors.push_back(R_HIP_ROLL);
		motors.push_back(R_HIP_PITCH);
		motors.push_back(R_KNEE);
		motors.push_back(R_ANKLE_PITCH);
		motors.push_back(R_ANKLE_ROLL);
		motors.push_back(L_HIP_YAW);
		motors.push_back(L_HIP_ROLL);
		motors.push_back(L_HIP_PITCH);
		motors.push_back(L_KNEE);
		motors.push_back(L_ANKLE_PITCH);
		motors.push_back(L_ANKLE_ROLL);
	}
	else
	{
		Logger::getInstance() << "Error while loading configuration file. Cannot initialize motors." << std::endl;
	}
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
	int iOffsetValue = 512;
	int value = (angle*dInvAngleConvertion)+iOffsetValue;
	return clamp(value, 21, 1002);
}

int MotorControl::Angle2Value(const int ID, const double angle)
{
	int value = (angle*dInvAngleConvertion)+motors[ID].offset;
	return clamp(value, motors[ID].minLimit, motors[ID].maxLimit);
}

double MotorControl::Value2Angle(const int value)
{
	int iOffsetValue = 512;
	int clampedValue = clamp(value, 21, 1002);
	return double(clampedValue-iOffsetValue)*dAngleConvertion;
}

double MotorControl::Value2Angle(const int ID, const int value)
{
	int clampedValue = clamp(value, motors[ID].minLimit, motors[ID].maxLimit);
	return double(clampedValue-motors[ID].offset)*dAngleConvertion;
}
