#include "MotorControl.h"
#include "WalkStatus.h"

#include "LinuxDARwIn.h"

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <sstream>
#include <iterator>

using namespace Robot;

const double uDt = 0.016; // Time step in second

void run(const std::string filename, const bool isUsingAlgorithm, const bool isMotorActivated)
{
   std::cout << "Starting program. Sleeping 4 sec\n";
   sleep(4);
  /* 
   // Framework initialization
   LinuxCM730 linux_cm730("/dev/ttyUSB0");
   CM730 cm730(&linux_cm730);
   if(cm730.Connect() == false)
   {
      std::cout << "Fail to connect CM-730!\n";
      return 0;
   }*/
   
    
   WalkStatus calculR(2);//, calculL(2);   
//   MotorControl motion = MotorControl( &cm730 );
   
   // Enable Torque
   if( !motion.SetTorque( MotorControl::ALL_LEGS ) )
   {
      std::cout << "SetTorque Failed\n";
      return;
   }

   // Initialise position 
   std::string strLine;
   std::ifstream file;
   
   file.open( filename.c_str() );
   std::vector<double> vPos;  

   // Set inital position
   if( !file.eof() )
   {
      getline( file, strLine );
      std::istringstream iss(strLine);

      std::copy(std::istream_iterator<double>(iss),
	        std::istream_iterator<double>(),
	        std::back_inserter(vPos));      
     
      /*calculR.getMotorPosition( vPos );
      std::vector<double>::iterator itr = vPos.begin();
      const std::vector<double>::iterator end = vPos.end();
      for( ; itr != end; itr++ )
      {
	vPos.push_back( -*itr );
      }
     */ 
		// TODO Rename InitPosition to GoToPosition
      if(isMotorActivated)
		{
			if( !motion.InitPosition( vPos, MotorControl::ALL_LEGS, 3000 ) )
	      {
				file.close(); 
				return;
	      }
		}
		if(isDebug && file.eof())
		{
			//TODO cout pos
		}
   } 
	else
	{
		file.close();
		return;
	}
   
   std::cout << "Beginning mouvement. Sleeping 1 sec\n";
   sleep(1);

	if(!isUsingAlgorithm)
	{
	   // Process mouvement with file as input
	   while( !file.eof() )
	   {
	      vPos.clear();
	      
	      getline( file, strLine );
	      std::istringstream iss(strLine);
	
	      std::copy(std::istream_iterator<double>(iss),
		        std::istream_iterator<double>(),
		        std::back_inserter(vPos));
	      
			if(isMotorActivated)
			{
		      if( file.eof() || !motion.SetPosition( vPos, MotorControl::ALL_LEGS ) )
		      {
					break;
	      	}
			}
			if(isDebug && file.eof())
			{
				//TODO cout pos
			}

      	usleep(uDt**1000*1000);
   	} 
	}
	else
	{ 
		//Process mouvement with algorithm

		//TODO remove HardCoded
	   const double tf = 3.1;
	   double dt = uDt;
		//const int numberOfExecution = 1;
   
   //for( int i = 0; i < numberOfExecution; i++ )
   //{  
      //if( i % 2 == 0 )
      //{
			calculR.initAllTrajParam(0.03, 0.02);
			//calculL.initAllTrajParam(-0.03, 0.00);
      //}
      //else
      //{
			//calculR.initAllTrajParam(-0.03, 0.00);
			//calculL.initAllTrajParam(0.03, 0.02);
      //}
      
      for( double time = 0.0; time <= tf + dt; time += dt )
      {
			// Right Leg movement
			if(isMotorActivated)
			{
				vPos.clear();
				motion.ReadPosition( vPos, MotorControl::RIGHT_LEGS );
			}

			calculR.Process( time, vPos );
			calculR.getMotorPosition( vPos );

			if(isMotorActivated)
			{
				if( !motion.SetPosition( vPos, MotorControl::RIGHT_LEGS ) )
				{
					break;
				}
			}
			if(isDebug)
			{
				//TODO cout pos
			}

			//TODO Left leg movement should be calculated in the calcul algorithm.
			/*
			// Left Leg movement
			vPos.clear();
			motion.ReadPosition( vPos, MotorControl::LEFT_LEGS );
			// Reverse motor angles
			std::vector<double>::iterator itr = vPos.begin();
			const std::vector<double>::iterator end = vPos.end();
			for( ; itr != end; itr++ )
			{
			    *itr = -*itr;
			}
			calculL.Process( time, vPos );
			calculL.getMotorPosition( vPos );
			// Reverse motor angles
			itr = vPos.begin();
			for( ; itr != end; itr++ )
			{
			    *itr = -*itr;
			}
			if( !motion.SetPosition( vPos, MotorControl::LEFT_LEGS ) )
			{
			    break;
			}*/
			
			usleep(dt*1000*1000);
      }   
   //}
	}
 
   std::cout << "Finishing program. Sleeping 4 sec\n";
   sleep(4);
   
   //file.close();
   std::cout << "Program finished\n";
   
   return;
}

int main(int argc, char* argv[])
{   
	//TODO Add argument for activate (RightLeg,LeftLeg,AllLeg) and check if file and calcul fit this..
	//TODO Add argument for number of loop...
	if(argc > 1)
	{
		std::string filename;
		bool isUsingAlgorithm(false), isMotorActivated(false), isDebug(false);

		if(argv[1] == "help")
		{
			std::cout << "First argument : File name of text file containing motor position. (Always needed for the initial position)" << std::endl;
			std::cout << "-ua : Use your algorithm." << std::endl
						 << "-am : Activate Motor." << std::endl
						 << "-de : Activate debug position sending to motor." << std::endl << std::endl;
			return 0;
		}	
		else
		{
			filename = argv[1];
		}

		for(int i = 2; i < argc; i++)
		{
			if(argv[i] == "-ua")
			{
				isUsingAlgorithm = true;
			}
			else if(argv[i] == "-am")
			{
				isMotorActivated = true;
			}
			else if(argv[i] == "-de")
			{
				isDebug = true;
			}
			else
			{
				std::cout << "Invalid argument" << std::endl;
				return 1;
			}
		}

		run(filename, isUsingAlgorithm, isMotorActivated);
	}
	else
	{
		std::cout << "No argument" << std::endl;
		return 1;
	}

  return 0;
}


