/*!
 * \file RobotFoot.cpp
 * \brief The main file of the project
 * \authors Mitchel Labonté and Mickael Paradis
 * \version 0.2
 */
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/chrono.hpp>
#include <memory> // shared_ptr

// TODO Remove from here
#include "ImageProcessing/Camera.h"
#include "ImageProcessing/ColorFinder.h"
#include "ImageProcessing/ObjectTracker.h"
// TODO to here

#include "Utilities/XmlParser.h"
#include "Utilities/logger.h"
#include "Utilities/ThreadManager.h"
#include "Control/MotorControl_2.h"
#include "ImageProcessing/HeadControlTask.h"
#include "Demo/StaticWalking/StaticWalk.h"

/*!
 * \brief Use for the demo
 *
 *  Track the ball and use feedback for controlling the motor of the head
 *
 *  \param mc : An instance of the micro controller
 */
/*void hardSet(STM32F4& mc)
{
	using boost::filesystem::path;

	Logger::getInstance().addStream(std::cout);
	Logger::getInstance() << "Starting tracking demo." << std::endl;

	// Load config
	Logger::getInstance() << "Loading configuration file..." << std::endl;
	XmlParser config;
	if (!config.loadFile("config.xml"))
	{
		Logger::getInstance() << "Error while loading configuration file." << std::endl;
		return;
	}
	path basePath = XmlPath::Root / XmlPath::Motion / XmlPath::Motors / XmlPath::Head;

	int pan = config.getIntValue(basePath / XmlPath::Pan);
	int tilt = config.getIntValue(basePath / XmlPath::Tilt);

	mc.setTorque(pan, STM32F4::TorqueOn);
	mc.setTorque(tilt, STM32F4::TorqueOn);

	Logger::getInstance() << "Force set process started" << std::endl;
	int m;
	while(true)
	{
		Logger::getInstance() << "Set pan :" << std::endl;
		std::cin >> m;
		mc.setMotor(pan, m);

		Logger::getInstance() << "Set tilt :" << std::endl;
		std::cin >> m;
		mc.setMotor(tilt, m);

		if((cvWaitKey(10) & 255) == 27) break;
	}
}*/

int main(int argc, char * argv[])
{ 
    // Add io stream to Logger
    Logger::getInstance().addStream(std::cout);

    // Load config file
    Logger::getInstance() << "Loading configuration file..." << std::endl;
    XmlParser config;
    if (!config.loadFile("config/config.xml")) 
    {
        Logger::getInstance(Logger::LogLvl::ERROR) << "Error while loading configuration file." << std::endl;
        std::exit(1);
    }

    // Set logging level  
    Logger::getInstance().setLogLvl(config.getStringValue(XmlPath::Root / "Logging" / "LogLvl"));

    // Thread Manager
    try
    {
        // Init IO_service for ThreadManager
        boost::asio::io_service boost_io;
        std::shared_ptr<ThreadManager> threadManager_ptr(new ThreadManager(boost_io, config));
        //threadManager_ptr->create(70, boost::bind(&boost::asio::io_service::run, &boost_io));

        std::shared_ptr<MotorControl> motorControl_ptr(new MotorControl(threadManager_ptr, config));

        
        // Starting Head task
        bool isTracking = config.getIntValue(XmlPath::Root / XmlPath::ImageProcessing);
        bool isKicking  = config.getIntValue(XmlPath::Root / XmlPath::Motion);
        if (isTracking)
        {
        	HeadControlTask headControlTask(threadManager_ptr, config, motorControl_ptr);
        	threadManager_ptr->create(50, [headControlTask]() mutable {headControlTask.run();},ThreadManager::Task::HEAD_CONTROL);
        	//threadManager_ptr->create(50, boost::bind(&HeadControlTask::run, &headControlTask),ThreadManager::Task::HEAD_CONTROL);
        	//headControlTask.run();
        }

        StaticWalk staticWalk(threadManager_ptr, motorControl_ptr);
        if (isKicking)
        {
        	// Init Walk task
        	staticWalk.init("config/input.txt", false, true, true);
        
        	// Head Task
        	staticWalk.initPosition(7000);

        	threadManager_ptr->attach(threadManager_ptr->create(90, boost::bind(&StaticWalk::run, &staticWalk,
                                      config.getIntValue(XmlPath::Root / XmlPath::Motion / XmlPath::IterationTimeMs)),
                                      ThreadManager::Task::LEGS_CONTROL));
        }

        if (isTracking)
        	threadManager_ptr->attach(ThreadManager::Task::HEAD_CONTROL);

        //threadManager_ptr->create(90, boost::bind(&MotorControl::run, &motorControl), ThreadManager::Task::MOTOR_CONTROL);
        //threadManager_ptr->timer(); // Start timer
        
        Logger::getInstance() << "END" << std::endl;
    }
    catch (std::exception& e)
    {
        Logger::getInstance(Logger::LogLvl::ERROR) << "Exception while initialising ThreadManager : " << e.what() << std::endl;
    }
    return 0;
}
