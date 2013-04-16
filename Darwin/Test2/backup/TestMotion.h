#ifndef TEST_MOTION_H
#define TEST_MOTION_H

#include "CM730.h"

#include <vector>

using namespace Robot;

class TestMotion
{
public:
   enum TestOption {
      TEST_LEGS = 0,
      NUM_TEST
   };

   TestMotion( CM730* cm730, TestOption option );
   ~TestMotion();

   bool init();
   bool unInit();
  
   //msTotalTime is the total time in ms before it arrived to its initial position.
   bool initPosition( const std::vector<double>& vPos, double msTotalTime = 10000.0 );	
   bool Process( const std::vector<double>& vPos );	

   bool readPosition( std::vector<double>& pos );
private:
   
   struct Configuration
   {
      std::vector<int> joints;
      int P;
      int I;
      int D;    
   };
   
   bool initLegs();
   
   
   bool setPosition( const std::vector<double>& pos);
   
   unsigned int getJointNum();
   
   CM730* _cm730;
   TestOption _option;
   Configuration _config[NUM_TEST];
   
};

#endif  //TEST_MOTION_H
