/**
******************************************************************************^M
* @file    LegMotion.cpp
* @author  Camille Hébert & Antoine Rioux
* @date    2013-11-19
* @brief   Class to control the leg movements
******************************************************************************^M
*/

#include "LegMotion.h"

#include <iostream>
#include <fstream>
#include <boost/thread.hpp>
#include <sstream>
#include <iterator>

#include "MotionControl.h"
#include "Trajectory.h"


LegMotion::LegMotion(std::shared_ptr<ThreadManager> threadManager_ptr, std::shared_ptr<MotorControl> mc_ptr):
    m_threadManager(threadManager_ptr),
    m_motion(mc_ptr)
{
	m_motionControl = new MotionControl();
}

LegMotion::~LegMotion()
{
	delete m_motionControl;
}

void LegMotion::InitWalk(Eigen::Vector2f destination, Eigen::Vector2f startingFeetAngles, Eigen::Vector2f destinationFeetAngles,
		const bool isMotorActivated, const bool isStandAlone, const int msInitializationTime, float stepTime)
{
	m_bIsMotorActivated = isMotorActivated;
	m_bIsUsingAlgorithm = true;
	m_bIsStandAlone = isStandAlone;

	Trajectory* traj = new Trajectory();
	m_trajectoryMatrix = traj->GenerateWalk(Eigen::Vector2f(0, 0), destination,
			destinationFeetAngles, startingFeetAngles, stepTime);

	m_vInitialPosition = m_motionControl->GetInitialQPosition();

	//Set the initial position
	InitPosition(msInitializationTime);
}

void LegMotion::InitKick(const bool isMotorActivated, const bool isStandAlone, const int msInitializationTime, float kickTime)
{
	m_bIsMotorActivated = isMotorActivated;
	m_bIsUsingAlgorithm = true;
	m_bIsStandAlone = isStandAlone;

	/*Trajectory* traj = new Trajectory();

	m_trajectoryMatrix = traj->GenerateKick(Eigen::Vector2f(0, 0), destination,
			destinationFeetAngles, startingFeetAngles, stepTime);*/

    //m_vInitialPosition = motioncontrol->getInitialQPosition();

	//Set the initial position
	InitPosition(msInitializationTime);
}

void LegMotion::Init(const std::string filename, const bool isMotorActivated, const bool isStandAlone, const int msInitializationTime)
{
	m_bIsMotorActivated = isMotorActivated;
	m_bIsUsingAlgorithm = false;
	m_bIsStandAlone = isStandAlone;

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

        m_vPosition.push_back(vPos);
        getline( file, strLine );

    }

    m_itrPos = m_vPosition.begin();
   // m_vInitialPosition = std::vector<float>(*m_vPosition.begin()); //Erreurs possible!!!
    m_vInitialPosition = (*m_vPosition.begin());
    m_itrEnd = m_vPosition.end();

	//Set the initial position
	InitPosition(msInitializationTime);
}

void LegMotion::InitPosition(const int msInitializationTime)
{
    // Enable Torque
    if( m_bIsMotorActivated && !m_motion->SetTorque(true, MotorControl::Config::ALL_LEGS ) )
    {
        Logger::getInstance() << "SetTorque Failed\n";
        return;
    }

    if(m_bIsMotorActivated && !m_vInitialPosition.empty())
    {
        if( !m_motion->InitPositions( m_vInitialPosition, MotorControl::Config::ALL_LEGS, msInitializationTime ) )
        {
           Logger::getInstance() << "InitPosition Failed\n";
           return;
        }
    }

}

void LegMotion::Run(double msDt)
{
    try
    {
		if(!m_bIsUsingAlgorithm)
		{
			m_threadManager->wait();
			boost::chrono::system_clock::time_point start = boost::chrono::system_clock::now();
			// Process mouvement with file as input
			for(;m_itrPos != m_itrEnd; ++m_itrPos)
			{
				if (!m_bIsStandAlone)
				{
					boost::this_thread::interruption_point();
					Logger::getInstance(Logger::LogLvl::DEBUG) << "StaticWalk : wait for MotorControl" << std::endl;
					m_threadManager->wait();
				}

				boost::chrono::duration<double> sec = boost::chrono::system_clock::now() - start;
				Logger::getInstance(Logger::LogLvl::DEBUG) << "took " << sec.count() << " seconds" << std::endl;

				start = boost::chrono::system_clock::now();

				if (m_bIsMotorActivated)
				{
					if (m_bIsStandAlone)
					{
						Logger::getInstance() << "StaticWalk : HardSet" << std::endl;
						m_motion->HardSet( *m_itrPos, MotorControl::Config::ALL_LEGS );
						Logger::getInstance() << "StaticWalk : After HardSet" << std::endl;
					}
					else
					{
						if(!m_motion->SetPositions( *m_itrPos, MotorControl::Config::ALL_LEGS ) )
						{
							Logger::getInstance() << "SetPosition Failed\n";
							break;
						}
					}
				}
				for(std::vector<double>::iterator it = m_itrPos->begin(); it != m_itrPos->end(); ++it)
				{
					Logger::getInstance() << *it << " ";
				}
				Logger::getInstance() << std::endl;

				if (m_bIsStandAlone)
				{
					usleep(msDt*1000);
				}
				else
				{
					m_threadManager->resume(ThreadManager::Task::MOTOR_CONTROL);
					Logger::getInstance(Logger::LogLvl::DEBUG) << "StaticWalk : Iteration done" << std::endl;
					m_threadManager->resume(ThreadManager::Task::MOTOR_CONTROL);
				}
			}
		}
		else
		{
			std::vector<double> motorsPosition;
			for(int i = 0; i < m_trajectoryMatrix.rows(); ++i)
			{
				//boost::chrono::system_clock::time_point start = boost::chrono::system_clock::now();i
				boost::this_thread::interruption_point();
				m_threadManager->wait();

				// Right Leg movement
				if(m_bIsMotorActivated)
				{
					motorsPosition.clear();
					m_motion->HardGet( motorsPosition, MotorControl::Config::ALL_LEGS );
				}

				//read motors
				motorsPosition = m_motionControl->UpdateQ((Eigen::VectorXf)m_trajectoryMatrix.row(i), motorsPosition);

				//set motors
				if(m_bIsMotorActivated)
				{
					m_motion->HardSet( motorsPosition, MotorControl::Config::ALL_LEGS );
					if( motorsPosition.empty() )
					{
						break;
					}
				}

				m_threadManager->resume(ThreadManager::Task::MOTOR_CONTROL);
			}
		}
    }
    catch(boost::thread_interrupted const &e)
    {
        Logger::getInstance() << "LEGS_CONTROL task Interrupted. " << std::endl;
    }
}

