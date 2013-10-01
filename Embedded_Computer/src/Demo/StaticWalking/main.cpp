#include "WalkStatus.h"

#include "../../Utilities/logger.h"
#include "../../Control/STM32F4.h"
#include "../../Control/MotorControl.h"

#include <boost/asio.hpp>
#include <boost/thread.hpp>


#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <sstream>
#include <iterator>

const double uDt = 0.016; // Time step in second
const int msInitializationTime = 10000;

void run(const std::string filename, const bool isUsingAlgorithm, const bool isMotorActivated)
{
   std::cout << "Starting program. Sleeping 4 sec\n";
   sleep(4);
   
	boost::asio::io_service io; 
	STM32F4 mc(std::string("/dev/ttyACM0"), io);
	boost::thread t(boost::bind(&boost::asio::io_service::run, &io));

   WalkStatus calculR(2);//, calculL(2);   
	MotorControl motion = MotorControl( &mc );
   
   // Enable Torque
   if( isMotorActivated && !motion.SetTorque( MotorControl::RIGHT_LEGS ) )
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
     
		// TODO Rename InitPosition to GoToPosition
      if(isMotorActivated)
		{
			if( !motion.InitPosition( vPos, MotorControl::RIGHT_LEGS, msInitializationTime ) )
	      {
				file.close(); 
				return;
	      }
		}
		for(std::vector<double>::iterator it = vPos.begin(); it != vPos.end(); ++it)
		{
			Logger::getInstance() << *it << " ";
		}
		Logger::getInstance() << std::endl;
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
		      if( file.eof() || !motion.SetPosition( vPos, MotorControl::RIGHT_LEGS ) )
		      {
					break;
	      	}
			}
			for(std::vector<double>::iterator it = vPos.begin(); it != vPos.end(); ++it)
			{
				Logger::getInstance() << *it << " ";
			}
			Logger::getInstance() << std::endl;

      	usleep(uDt*1000*1000);
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
			/*if(isMotorActivated)
			{
				vPos.clear();
				motion.ReadPosition( vPos, MotorControl::RIGHT_LEGS );
			}*/

			calculR.Process( time, vPos );
			calculR.getMotorPosition( vPos );

			/*if(isMotorActivated)
			{
				if( !motion.SetPosition( vPos, MotorControl::RIGHT_LEGS ) )
				{
					break;
				}
			}*/
			for(std::vector<double>::iterator it = vPos.begin(); it != vPos.end(); ++it)
			{
				Logger::getInstance() << *it << " ";
			}
			Logger::getInstance() << std::endl;
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
   
   file.close();
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
		bool isUsingAlgorithm(false), isMotorActivated(false);

      const std::string help = "-help";
		const std::string usingAlgorithm = "-ua";
		const std::string motorActivated = "-am";
		const std::string debug = "-de";
 
		if(argv[1] == help)
		{
			std::cout << "First argument : File name of text file containing motor position. (Always needed for the initial position)" << std::endl;
			std::cout << "-ua : Use your algorithm." << std::endl
						 << "-am : Activate Motor." << std::endl
						 << "-de : Show debug message." << std::endl;
			return 0;
		}	
		else
		{
			filename = argv[1];
		}

		for(int i = 2; i < argc; i++)
		{
			if(argv[i] == usingAlgorithm)
			{
				isUsingAlgorithm = true;
			}
			else if(argv[i] == motorActivated)
			{
				isMotorActivated = true;
			}
			else if(argv[i] == debug)
			{
				Logger::getInstance().addStream(std::cout);
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


