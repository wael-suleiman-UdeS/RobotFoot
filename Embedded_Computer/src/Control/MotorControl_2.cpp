#include "Control/MotorControl_2.h"
#include "Utilities/logger.h"

#include <iostream>
#include <iterator>
#include <algorithm>
#include <functional>
#include <boost/algorithm/clamp.hpp>
#include <boost/asio.hpp>

using boost::filesystem::path;

//#define DEBUG_TEST_MOTION

using boost::algorithm::clamp;


#define DANGER_TEST_MOTION

namespace
{
	const double dAngleConvertion = 0.325;
	const double dInvAngleConvertion = 1/dAngleConvertion;
}

MotorControl::MotorControl( ThreadManager *threadManager, const XmlParser &config ) :
 _threadManager(threadManager)
{
    try
    {
        // Init USB interface with STM32F4
        Logger::getInstance() << "Initializing USB interface..." << std::endl;
        boost::asio::io_service boost_io;
        std::string port_name = config.getStringValue(XmlPath::Root / "USB_Interface" / "TTY");
        _stm32f4 = new STM32F4(port_name, boost_io);
        threadManager->create(50, "boost::io_service",  boost::bind(&boost::asio::io_service::run, &boost_io));
    }
    catch (std::exception& e)
    {
        Logger::getInstance(Logger::LogLvl::ERROR) << "Exception in MotorControl.cpp while initialising USB interface : " << e.what() << std::endl;
    }

    std::map<string, Motor> motorsMap;
    InitializeMotors(motorsMap, config);

    std::vector<Motor> joints;

    //Right Leg
    joints.push_back(motorsMap.find("R_HIP_YAW")->second);
    joints.push_back(motorsMap.find("R_HIP_ROLL")->second);
    joints.push_back(motorsMap.find("R_HIP_PITCH")->second);
    joints.push_back(motorsMap.find("R_KNEE")->second);
    joints.push_back(motorsMap.find("R_ANKLE_PITCH")->second);
    joints.push_back(motorsMap.find("R_ANKLE_ROLL")->second);
    //Left leg
    joints.push_back(motorsMap.find("L_HIP_YAW")->second);
    joints.push_back(motorsMap.find("L_HIP_ROLL")->second);
    joints.push_back(motorsMap.find("L_HIP_PITCH")->second);
    joints.push_back(motorsMap.find("L_KNEE")->second);
    joints.push_back(motorsMap.find("L_ANKLE_PITCH")->second);
    joints.push_back(motorsMap.find("L_ANKLE_ROLL")->second);

    _leg_config[ALL_LEGS].joints = joints;
    //_config[ALL_LEGS].P = JointData::P_GAIN_DEFAULT;
    //_config[ALL_LEGS].I = JointData::I_GAIN_DEFAULT;
    //_config[ALL_LEGS].D =JointData::D_GAIN_DEFAULT;

    joints.clear();
    //Right Leg
    joints.push_back(motorsMap.find("R_HIP_YAW")->second);
    joints.push_back(motorsMap.find("R_HIP_ROLL")->second);
    joints.push_back(motorsMap.find("R_HIP_PITCH")->second);
    joints.push_back(motorsMap.find("R_KNEE")->second);
    joints.push_back(motorsMap.find("R_ANKLE_PITCH")->second);
    joints.push_back(motorsMap.find("R_ANKLE_ROLL")->second);

    _leg_config[RIGHT_LEGS].joints = joints;
    //_config[RIGHT_LEGS].P = JointData::P_GAIN_DEFAULT;
    //_config[RIGHT_LEGS].I = JointData::I_GAIN_DEFAULT;
    //_config[RIGHT_LEGS].D =JointData::D_GAIN_DEFAULT;

    joints.clear();
    //Left leg
    joints.push_back(motorsMap.find("L_HIP_YAW")->second);
    joints.push_back(motorsMap.find("L_HIP_ROLL")->second);
    joints.push_back(motorsMap.find("L_HIP_PITCH")->second);
    joints.push_back(motorsMap.find("L_KNEE")->second);
    joints.push_back(motorsMap.find("L_ANKLE_PITCH")->second);
    joints.push_back(motorsMap.find("L_ANKLE_ROLL")->second);
 
    _leg_config[LEFT_LEGS].joints = joints;
    //_config[LEFT_LEGS].P = JointData::P_GAIN_DEFAULT;
    //_config[LEFT_LEGS].I = JointData::I_GAIN_DEFAULT;
    //_config[LEFT_LEGS].D =JointData::D_GAIN_DEFAULT;
}

MotorControl::~MotorControl()
{

}

//Returns a map of joint name and Motor struct
void MotorControl::InitializeMotors(std::map<std::string, Motor> &motorsMap, const XmlParser &config)
{
    std::map<std::string, path> pathsMap;

    pathsMap.insert(std::make_pair("R_HIP_YAW", XmlPath::LegsMotors / XmlPath::R_HIP_YAW));
	pathsMap.insert(std::make_pair("L_HIP_YAW", XmlPath::LegsMotors / XmlPath::L_HIP_YAW));
	pathsMap.insert(std::make_pair("R_HIP_ROLL", XmlPath::LegsMotors / XmlPath::R_HIP_ROLL));
	pathsMap.insert(std::make_pair("L_HIP_ROLL", XmlPath::LegsMotors / XmlPath::L_HIP_ROLL));
	pathsMap.insert(std::make_pair("R_HIP_PITCH", XmlPath::LegsMotors / XmlPath::R_HIP_PITCH));
	pathsMap.insert(std::make_pair("L_HIP_PITCH", XmlPath::LegsMotors / XmlPath::L_HIP_PITCH));
	pathsMap.insert(std::make_pair("R_KNEE", XmlPath::LegsMotors / XmlPath::R_KNEE));
	pathsMap.insert(std::make_pair("L_KNEE", XmlPath::LegsMotors / XmlPath::L_KNEE));
	pathsMap.insert(std::make_pair("R_ANKLE_PITCH", XmlPath::LegsMotors / XmlPath::R_ANKLE_PITCH));
	pathsMap.insert(std::make_pair("L_ANKLE_PITCH", XmlPath::LegsMotors / XmlPath::L_ANKLE_PITCH));
	pathsMap.insert(std::make_pair("R_ANKLE_ROLL", XmlPath::LegsMotors / XmlPath::R_ANKLE_ROLL));
	pathsMap.insert(std::make_pair("L_ANKLE_ROLL", XmlPath::LegsMotors / XmlPath::L_ANKLE_ROLL));

	Motor motor;
	for(auto it = pathsMap.begin(); it != pathsMap.end(); ++it)
	{
	    motor.id       = config.getIntValue(it->second / XmlPath::MotorID);
		motor.offset   = config.getIntValue(it->second / XmlPath::Offset);
		motor.minLimit = config.getIntValue(it->second / XmlPath::LimitMin);
		motor.maxLimit = config.getIntValue(it->second / XmlPath::LimitMax);	
		motorsMap.insert(std::make_pair(it->first, motor));
	}
}

bool MotorControl::SetTorque( const Option option )
{
   bool status = true;

   std::vector<Motor>::const_iterator itr = _leg_config[ option ].joints.begin();
   const std::vector<Motor>::const_iterator end = _leg_config[ option ].joints.end();

   for( ; itr != end && status ; itr++ )
   {
     // TODO : Grab status
      //status &=
		Motor motor = *itr;
		_stm32f4->setTorque(motor.id,STM32F4::TorqueOn);
      //_cm730->WriteByte(*itr, MX28::P_P_GAIN, JointData::P_GAIN_DEFAULT, 0);
      //_cm730->WriteByte(*itr, MX28::P_I_GAIN, JointData::I_GAIN_DEFAULT, 0);
      //_cm730->WriteByte(*itr, MX28::P_D_GAIN, JointData::D_GAIN_DEFAULT, 0);
		//TODO : temporary test 
	//	sleep(1);
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
       Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"InitPosition\" : Joint number is invalid." 
                                                  << std::endl;
#endif
      return false;
   }

#ifdef DEBUG_TEST_MOTION
   Logger::getInstance(Logger::LogLvl::DEBUG) << "Setting initial position : \n";
   std::copy(desiredPos.begin(), desiredPos.end(), std::ostream_iterator<double>(std::cout, " "));
   Logger::getInstance(Logger::LogLvl::DEBUG) << std::endl;
#endif   

   std::vector<double> pos;
   if( !ReadPosition( pos, option ) )
   {
#ifdef DEBUG_TEST_MOTION      
       Logger::getInstance(Logger::LogLvl::ERROR) << "Error reading motor position" 
                                                  << std::endl;
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
       Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"SetPosition\" : Joint number is invalid."
                                                  << std::endl;
#endif
      return false;
   }

   bool status = true;

#ifdef DANGER_TEST_MOTION   
   auto itrJoint = _leg_config[ option ].joints.begin();
   const auto endJoint = _leg_config[ option ].joints.end();
   auto itrPos = pos.begin();
   const auto endPos = pos.end();

   for( ; itrJoint != endJoint && itrPos != endPos && status ; itrJoint++, itrPos++ )
   {
		Motor motor = *itrJoint;
		_stm32f4->setMotor(motor.id,Angle2Value(motor, *itrPos));
   }
#endif

#ifdef DEBUG_TEST_MOTION
      Logger::getInstance(Logger::LogLvl::DEBUG) << "";
      std::copy(pos.begin(), pos.end(), std::ostream_iterator<double>(std::cout, " "));
      Logger::getInstance(Logger::LogLvl::DEBUG) << std::endl;
#endif

   return status;
}

bool MotorControl::ReadPosition( std::vector<double>& pos, const Option option )
{
   bool status = true;
   int value;

   auto itr = _leg_config[ option ].joints.begin();
   const auto end = _leg_config[ option ].joints.end();

   for( ; itr != end && status ; itr++ )
   {
	Motor motor = *itr;
	value = _stm32f4->read(motor.id);
	pos.push_back( Value2Angle(motor, value) );
   }

   return status;
}

unsigned int MotorControl::getJointNum( const Option option )
{
   return _leg_config[ option ].joints.size();
}

int MotorControl::Angle2Value(const double angle)
{
	int iOffsetValue = 512;
	int value = (angle*dInvAngleConvertion)+iOffsetValue;
	return clamp(value, 21, 1002);
}

int MotorControl::Angle2Value(const Motor& motor, const double angle)
{
	int value = (angle*dInvAngleConvertion) + motor.offset;
	return clamp(value, motor.minLimit, motor.maxLimit);
}

double MotorControl::Value2Angle(const int value)
{
	int iOffsetValue = 512;
	int clampedValue = clamp(value, 21, 1002);
	return double(clampedValue-iOffsetValue)*dAngleConvertion;
}

double MotorControl::Value2Angle(const Motor& motor, const int value)
{
	int clampedValue = clamp(value, motor.minLimit, motor.maxLimit);
	return double(clampedValue - motor.offset)*dAngleConvertion;
}
