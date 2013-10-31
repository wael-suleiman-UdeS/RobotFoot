/**
******************************************************************************^M
* @file    StaticWalk.cpp
* @author  Mathieu Drapeau
* @date    2013-09-20
* @brief   Class to test mouvement on robot
******************************************************************************^M
*/

#include "StaticWalk.h" 

#include "WalkStatus.h"

#include <iostream>
#include <fstream>
#include <boost/thread.hpp>
#include <sstream>
#include <iterator>

StaticWalk::StaticWalk(std::shared_ptr<ThreadManager> threadManager_ptr, std::shared_ptr<MotorControl> mc_ptr):
    _threadManager(threadManager_ptr),
    _motion(mc_ptr)
{

}

StaticWalk::~StaticWalk()
{

}

void StaticWalk::init(const std::string filename, const bool isUsingAlgorithm, const bool isMotorActivated, const bool isStandAlone)
{
    bIsMotorActivated = isMotorActivated;
    bIsUsingAlgorithm = isUsingAlgorithm;
    bIsStandAlone = isStandAlone;
   
    std::string strLine;
    std::ifstream file;
    file.open( filename.c_str() );
    std::vector<double> vPos;  
    getline( file, strLine );
    
    while( !file.eof() )
    {
        vPos.clear();
        std::istringstream iss(strLine);

        std::copy(std::istream_iterator<double>(iss),
            std::istream_iterator<double>(),
            std::back_inserter(vPos));

        vPosition.push_back(vPos);
        getline( file, strLine );

    }

    itrPos = vPosition.begin();
    itrEnd = vPosition.end(); 
    /*for(std::vector<std::vector<double>>::iterator itr = vPosition.begin(); itr != vPosition.end(); ++itr)
    {
        for(std::vector<double>::iterator it = itr->begin(); it != itr->end(); ++it)
    	{
    		Logger::getInstance() << *it << " ";
        }
        Logger::getInstance() << std::endl;
    }*/
}

void StaticWalk::initPosition(const int msInitializationTime)
{
    // Enable Torque
    if( bIsMotorActivated && !_motion->SetTorque(true, MotorControl::Config::ALL_LEGS ) )
    {
        Logger::getInstance() << "SetTorque Failed\n";
        return;
    }
   
    if(bIsMotorActivated)
    {
        if( itrPos != itrEnd && !_motion->InitPositions( *itrPos, MotorControl::Config::ALL_LEGS, msInitializationTime ) )
        {
           Logger::getInstance() << "InitPosition Failed\n";
           return;
        }
    }

}

void StaticWalk::run(double msDt)
{
    try
    {
			if(!bIsUsingAlgorithm)
			{
				_threadManager->wait();
				boost::chrono::system_clock::time_point start = boost::chrono::system_clock::now();
				// Process mouvement with file as input
				for(;itrPos != itrEnd; ++itrPos)
				{
					if (!bIsStandAlone)
					{
						boost::this_thread::interruption_point();
						Logger::getInstance(Logger::LogLvl::DEBUG) << "StaticWalk : wait for MotorControl" << std::endl;
						_threadManager->wait();
					}

					boost::chrono::duration<double> sec = boost::chrono::system_clock::now() - start;
					Logger::getInstance(Logger::LogLvl::DEBUG) << "took " << sec.count() << " seconds" << std::endl;

					start = boost::chrono::system_clock::now();

					if (bIsMotorActivated)
					{
						if (bIsStandAlone)
						{
							Logger::getInstance() << "StaticWalk : HardSet" << std::endl;
							_motion->HardSet( *itrPos, MotorControl::Config::ALL_LEGS );
							Logger::getInstance() << "StaticWalk : After HardSet" << std::endl;
						}
						else
						{
							if(!_motion->SetPositions( *itrPos, MotorControl::Config::ALL_LEGS ) )
							{
								Logger::getInstance() << "SetPosition Failed\n";
								break;
							}
						}
					}
					for(std::vector<double>::iterator it = itrPos->begin(); it != itrPos->end(); ++it)
					{
						Logger::getInstance() << *it << " ";
					}
					Logger::getInstance() << std::endl;

					if (bIsStandAlone)
					{
						usleep(msDt*1000);
					}
					else
					{
						_threadManager->resume(ThreadManager::Task::MOTOR_CONTROL);
						Logger::getInstance(Logger::LogLvl::DEBUG) << "StaticWalk : Iteration done" << std::endl;
						_threadManager->resume(ThreadManager::Task::MOTOR_CONTROL);
					}
				}
			}
			else
			{
				//Process mouvement with algorithm

				//TODO remove HardCoded
				const double tf = 3.1;
				double dt = msDt;
				WalkStatus calculR(2);
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

				std::vector<double> vPos;
				for( double time = 0.0; time <= tf + dt; time += dt )
				{
					//boost::chrono::system_clock::time_point start = boost::chrono::system_clock::now();i
					boost::this_thread::interruption_point();
					_threadManager->wait();

					// Right Leg movement
					if(bIsMotorActivated)
					{
						vPos.clear();
						_motion->ReadPositions( vPos, MotorControl::Config::RIGHT_LEG );
					}

					calculR.Process( time, vPos );
					calculR.getMotorPosition( vPos );

					if(bIsMotorActivated)
					{
						if( !_motion->SetPositions( vPos, MotorControl::Config::RIGHT_LEG ) )
						{
							break;
						}
					}
					for(std::vector<double>::iterator it = vPos.begin(); it != vPos.end(); ++it)
					{
						Logger::getInstance() << *it << " ";
					}
					Logger::getInstance() << std::endl;

					//boost::chrono::duration<double> sec = boost::chrono::system_clock::now() - start;
					//std::cout << "took " << sec.count() << " seconds\n";
					////TODO Left leg movement should be calculated in the calcul algorithm.
	//            // Left Leg movement
	//            vPos.clear();
	//            _motion->ReadPositions( vPos, MotorControl::LEFT_LEG );
	//            // Reverse motor angles
	//            std::vector<double>::iterator itr = vPos.begin();
	//            const std::vector<double>::iterator end = vPos.end();
	//            for( ; itr != end; itr++ )
	//            {
	//             *itr = -*itr;
	//             }
	//             calculL.Process( time, vPos );
	//             calculL.getMotorPosition( vPos );
	//            // Reverse motor angles
	//            itr = vPos.begin();
	//            for( ; itr != end; itr++ )
	//            {
	//             *itr = -*itr;
	//             }
	//             if( !_motion->SetPositions( vPos, MotorControl::LEFT_LEG ) )
	//             {
	//             break;
	//             }

					//usleep(dt*1000*1000);
					_threadManager->resume(ThreadManager::Task::MOTOR_CONTROL);
				}
			//}
			}
    }
    catch(boost::thread_interrupted const &e)
    {
        Logger::getInstance() << "LEGS_CONTROL task Interrupted. " << std::endl;
    } 
}

