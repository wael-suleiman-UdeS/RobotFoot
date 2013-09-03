#include "MotorControl.h"

#include "CM730.h"
#include "JointData.h"
#include "MX28.h"

#include <iostream>
#include <iterator>
#include <algorithm>
#include <functional>

//#define DEBUG_TEST_MOTION
#define DANGER_TEST_MOTION

using namespace Robot;

MotorControl::MotorControl( CM730* cm730 ) :
 _cm730(cm730)
{  
   std::vector<int> joints;
   //Right Leg
   joints.push_back(JointData::ID_R_HIP_YAW);
   joints.push_back(JointData::ID_R_HIP_ROLL);
   joints.push_back(JointData::ID_R_HIP_PITCH);
   joints.push_back(JointData::ID_R_KNEE);
   joints.push_back(JointData::ID_R_ANKLE_PITCH);
   joints.push_back(JointData::ID_R_ANKLE_ROLL);
   //Left leg
   joints.push_back(JointData::ID_L_HIP_YAW);
   joints.push_back(JointData::ID_L_HIP_ROLL);
   joints.push_back(JointData::ID_L_HIP_PITCH);
   joints.push_back(JointData::ID_L_KNEE);
   joints.push_back(JointData::ID_L_ANKLE_PITCH);
   joints.push_back(JointData::ID_L_ANKLE_ROLL);
   
   _config[ALL_LEGS].joints = joints;
   _config[ALL_LEGS].P = JointData::P_GAIN_DEFAULT;
   _config[ALL_LEGS].I = JointData::I_GAIN_DEFAULT;
   _config[ALL_LEGS].D =JointData::D_GAIN_DEFAULT;  
      
   joints.clear();
   //Right Leg
   joints.push_back(JointData::ID_R_HIP_YAW);
   joints.push_back(JointData::ID_R_HIP_ROLL);
   joints.push_back(JointData::ID_R_HIP_PITCH);
   joints.push_back(JointData::ID_R_KNEE);
   joints.push_back(JointData::ID_R_ANKLE_PITCH);
   joints.push_back(JointData::ID_R_ANKLE_ROLL);
   
   _config[RIGHT_LEGS].joints = joints;
   _config[RIGHT_LEGS].P = JointData::P_GAIN_DEFAULT;
   _config[RIGHT_LEGS].I = JointData::I_GAIN_DEFAULT;
   _config[RIGHT_LEGS].D =JointData::D_GAIN_DEFAULT;
   
   joints.clear();
   //Left leg
   joints.push_back(JointData::ID_L_HIP_YAW);
   joints.push_back(JointData::ID_L_HIP_ROLL);
   joints.push_back(JointData::ID_L_HIP_PITCH);
   joints.push_back(JointData::ID_L_KNEE);
   joints.push_back(JointData::ID_L_ANKLE_PITCH);
   joints.push_back(JointData::ID_L_ANKLE_ROLL);
   
   _config[LEFT_LEGS].joints = joints;
   _config[LEFT_LEGS].P = JointData::P_GAIN_DEFAULT;
   _config[LEFT_LEGS].I = JointData::I_GAIN_DEFAULT;
   _config[LEFT_LEGS].D =JointData::D_GAIN_DEFAULT; 
   
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
      /*status &= */_cm730->WriteByte( *itr, MX28::P_TORQUE_ENABLE, 1, 0);
      _cm730->WriteByte(*itr, MX28::P_P_GAIN, JointData::P_GAIN_DEFAULT, 0);
      _cm730->WriteByte(*itr, MX28::P_I_GAIN, JointData::I_GAIN_DEFAULT, 0);
      _cm730->WriteByte(*itr, MX28::P_D_GAIN, JointData::D_GAIN_DEFAULT, 0);
   }
   
   return status;
}

bool MotorControl::DisableTorque( const Option option )
{
   bool status = true;
   
   std::vector<int>::const_iterator itr = _config[ option ].joints.begin();
   const std::vector<int>::const_iterator end = _config[ option ].joints.end();
   
   for( ; itr != end && status ; itr++ )
   {
     // TODO : Grab status
      /*status &= */_cm730->WriteByte( *itr, MX28::P_TORQUE_ENABLE, 0, 0);
   }
   
   return status;
}

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
      _cm730->WriteWord( *itrJoint, MX28::P_GOAL_POSITION_L, MX28::Angle2Value(*itrPos), 0);      
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
      status &= _cm730->ReadWord( *itr, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS;
      pos.push_back( MX28::Value2Angle(value) );
   }
   
   return status;
}

unsigned int MotorControl::getJointNum( const Option option )
{
   return _config[ option ].joints.size();
}