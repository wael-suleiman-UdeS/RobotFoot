#include "TestMotion.h"

#include "CM730.h"
#include "JointData.h"
#include "MX28.h"

#include <iostream>
#include <iterator>
#include <algorithm>
#include <functional>

using namespace Robot;

TestMotion::TestMotion( CM730* cm730, TestOption option ) :
 _cm730(cm730), _option(option)
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
   
   _config[TEST_LEGS].joints = joints;
   _config[TEST_LEGS].P = JointData::P_GAIN_DEFAULT;
   _config[TEST_LEGS].I = JointData::I_GAIN_DEFAULT;
   _config[TEST_LEGS].D =JointData::D_GAIN_DEFAULT;   
   
}

TestMotion::~TestMotion()
{

}

bool TestMotion::init()
{
   bool status;
   
   switch( _option )
   {
      case TEST_LEGS : 
	 status = initLegs();
	 break;
      default:
#ifdef DEBUG_TEST_MOTION
	 std::cout << "In function init, wrong test option\n";
#endif
	 return false;      	 
   }
   return status;
}

bool TestMotion::unInit()
{
   bool status = true;
   
   std::vector<int>::const_iterator itr = _config[ _option ].joints.begin();
   const std::vector<int>::const_iterator end = _config[ _option ].joints.end();
   
   for( ; itr != end && status ; itr++ )
   {
      /*status &= */_cm730->WriteByte( *itr, MX28::P_TORQUE_ENABLE, 0, 0);
   }
   
   return status;
}

//TODO: msDt should be a default parameter too
bool TestMotion::initPosition( const std::vector<double>& desiredPos, double msTotalTime /*= 10000.0*/ )
{
   if( desiredPos.size() != getJointNum() )
   {
#ifdef DEBUG_TEST_MOTION
      std::cout << "In function \"initPosition\" : Joint number is invalid."
                << std::endl;
#endif
      return false;
   }
   
#ifdef DEBUG_TEST_MOTION
   std::cout << "Setting initial position : \n";
   std::copy(desiredPos.begin(), desiredPos.end(), std::ostream_iterator<double>(std::cout, " "));
   std::cout << std::endl << "Sleeping 2 sec" << std::endl;   
#endif
   
   sleep(2);
   
   double msDt = 16;
   
   std::vector<double> pos;
   if( !readPosition(pos) )
   {
#ifdef DEBUG_TEST_MOTION      
      std::cout << "Error reading motor position";
#endif
      return false;
   }
   
   std::vector<double> posDt;
   
   std::vector<double>::const_iterator itrDesiredPos = desiredPos.begin();
   // Could be const
   const std::vector<double>::const_iterator endDesiredPos = desiredPos.end();
   std::vector<double>::iterator itrPos = pos.begin();
   // Could be const
   const std::vector<double>::iterator endPos = pos.end();
   
   // Calcul position dt for each motor
   for( ; itrDesiredPos != endDesiredPos || itrPos != endPos; itrDesiredPos++, itrPos++ )
   {
      posDt.push_back( (*itrDesiredPos - *itrPos)/(msTotalTime/msDt) );
   }
   
   // Loop to add position dt to motor position and send command to motor
   for(double t = 0.0; t < msTotalTime; t+=msDt)
   {
      std::transform (pos.begin(), pos.end(), posDt.begin(), pos.begin(), std::plus<double>());
      
      setPosition(pos);
      
      usleep(msDt*1000);
   }
   
   return true;
}

bool TestMotion::Process( const std::vector<double>& vPos )
{
   if( vPos.size() != getJointNum() )
   {
#ifdef DEBUG_TEST_MOTION
      std::cout << "In function \"Process\" : Joint number is invalid." 
                << std::endl;
#endif
      return false;
   }

#ifdef DEBUG_TEST_MOTION
   std::copy(vPos.begin(), vPos.end(), std::ostream_iterator<double>(std::cout, " "));
   std::cout << std::endl;
#endif
   //TODO: SetMotor (pos); 
   
   return true;
}

bool TestMotion::initLegs()
{
   bool status = true;
   
   std::vector<int>::const_iterator itr = _config[ _option ].joints.begin();
   const std::vector<int>::const_iterator end = _config[ _option ].joints.end();
   
   for( ; itr != end && status ; itr++ )
   {
      /*status &= */_cm730->WriteByte( *itr, MX28::P_TORQUE_ENABLE, 1, 0);
      //TODO: Should set PID
   }
   
   return status;
}

bool TestMotion::readPosition( std::vector<double>& pos )
{
   bool status = true;
   int value;
   
   std::vector<int>::const_iterator itr = _config[ _option ].joints.begin();
   const std::vector<int>::const_iterator end = _config[ _option ].joints.end();
   
   for( ; itr != end && status ; itr++ )
   {
      status &= _cm730->ReadWord( *itr, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS;
      pos.push_back( MX28::Value2Angle(value) );
   }
   
   return status;
}

bool TestMotion::setPosition( const std::vector<double>& pos)
{
   bool status = true;
   
#ifdef DANGER_TEST_MOTION   
   
   std::vector<int>::const_iterator itrJoint = _config[ _option ].joints.begin();
   const std::vector<int>::const_iterator endJoint = _config[ _option ].joints.end();
   std::vector<double>::const_iterator itrPos = pos.begin();
   const std::vector<double>::const_iterator endPos = pos.end();
   
   for( ; itrJoint != endJoint && itrPos != endPos && status ; itrJoint++, itrPos++ )
   {
      _cm730->WriteWord( *itrJoint, MX28::P_PRESENT_POSITION_L, MX28::Angle2Value(*itrPos), 0);      
   }
#endif

#ifdef DEBUG_TEST_MOTION
      std::copy(pos.begin(), pos.end(), std::ostream_iterator<double>(std::cout, " "));
      std::cout << std::endl;
#endif
   
   return status;

   
}

unsigned int TestMotion::getJointNum()
{
   return _config[ _option ].joints.size();
}