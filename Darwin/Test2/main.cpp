#include "TestMotion.h"

#include "LinuxDARwIn.h"

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <sstream>
#include <iterator>

#include <signal.h>
#include <cstdlib>
#include <sys/time.h>

using namespace Robot;

const std::string fileName = "Test.txt";
const int uDt = 8000; // Time step in micro second

struct itimerval oldTimer;
struct itimerval newTimer;

volatile bool isAlarmReady = false;

void
catch_alarm (int sig)
{
  isAlarmReady = true;
}

int main()
{   
   std::cout << "Starting program. Sleeping 4 sec\n";
   sleep(4);
   
   LinuxCM730 linux_cm730("/dev/ttyUSB0");
   CM730 cm730(&linux_cm730);
   if(cm730.Connect() == false)
   {
      std::cout << "Fail to connect CM-730!\n";
      return EXIT_FAILURE;
   }
   
   std::string strLine;
   std::ifstream file;
   
   file.open( fileName.c_str() );
   
   TestMotion motion = TestMotion( &cm730, TestMotion::TEST_LEGS );
   if( !motion.init() )
   {
      std::cout << "Initialization Failed\n";
      motion.unInit();
      file.close(); 
      return EXIT_FAILURE;
   }
   
   if( !file.eof() )
   {
      std::vector<double> vPos;
      
      getline( file, strLine );
      std::istringstream iss(strLine);

      std::copy(std::istream_iterator<double>(iss),
	        std::istream_iterator<double>(),
	        std::back_inserter(vPos));      
      
      if( !motion.initPosition( vPos ) )
      {
	 motion.unInit();
	 file.close(); 
	 return EXIT_FAILURE;
      }
   }
   
   signal (SIGALRM, catch_alarm);
   newTimer.it_interval.tv_sec = 0;
   newTimer.it_interval.tv_usec = uDt;
   newTimer.it_value.tv_sec = 0;
   newTimer.it_value.tv_usec = uDt;

   oldTimer.it_interval.tv_sec = 0;
   oldTimer.it_interval.tv_usec = 0;
   oldTimer.it_value.tv_sec = 0;
   oldTimer.it_value.tv_usec = 0;

   if (setitimer (ITIMER_REAL, &newTimer, &oldTimer) < 0)
   {
      std::cout << "Timer init failed.\n";
      motion.unInit();
      file.close(); 
      return EXIT_FAILURE;
   }

   bool isFinishCalculating = true;
   
   while( !file.eof() )
   {
      while( !isAlarmReady )
      {	 
      }
      isAlarmReady = false;
      if( !isFinishCalculating )
      {
	 std::cout << "Calcul did not finish in time.\n";
	 motion.unInit();
	 file.close(); 
	 return EXIT_FAILURE;
      }
      
      isFinishCalculating = false;
      std::vector<double> vPos;
      
      getline( file, strLine );
      std::istringstream iss(strLine);

      std::copy(std::istream_iterator<double>(iss),
	        std::istream_iterator<double>(),
	        std::back_inserter(vPos));
      
      if( !motion.Process( vPos ) && file.eof() )
      {
	 motion.unInit();
	 file.close(); 
	 return EXIT_FAILURE;
      }
      
      isFinishCalculating = true;
   }
   
   
   std::cout << "Finishing program. Sleeping 4 sec\n";
   sleep(4);
   
   motion.unInit();
   file.close();
   
   return EXIT_SUCCESS;	
}


