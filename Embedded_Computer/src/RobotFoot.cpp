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

#include "Utilities/XmlParser.h"
#include "Utilities/logger.h"
#include "Utilities/ThreadManager.h"
#include "Control/MotorControl_2.h"
#include "ImageProcessing/HeadControlTask.h"
//#include "Demo/StaticWalking/StaticWalk.h"
#include "LegMotion/LegMotion.h"

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

    try
    {
        // Init IO_service for ThreadManager and MotorControl
        boost::asio::io_service boost_io;
        std::shared_ptr<ThreadManager> threadManager_ptr(new ThreadManager(boost_io, config));
        std::shared_ptr<MotorControl> motorControl_ptr(new MotorControl(threadManager_ptr, config, boost_io));
        threadManager_ptr->create(80, [&boost_io]() mutable { boost_io.run(); }); 

        bool isTracking = config.getIntValue(XmlPath::Root / XmlPath::ImageProcessing);
        bool isMoving   = config.getIntValue(XmlPath::Root / XmlPath::Motion);
        if (isTracking)
        {
            // Starting Head task
        	HeadControlTask headControlTask(threadManager_ptr, config, motorControl_ptr);
        	threadManager_ptr->create(50, [headControlTask]() mutable { headControlTask.run(); }, ThreadManager::Task::HEAD_CONTROL);
        }

        if (isMoving)
        {
            //StaticWalk staticWalk(threadManager_ptr, motorControl_ptr);
            std::shared_ptr<LegMotion> legMotion_ptr(new LegMotion(threadManager_ptr, motorControl_ptr, config));
        	Eigen::Vector2f pointD(1, 0);
        	Eigen::Vector2f startAngle(0, 0);
        	Eigen::Vector2f endAngle(0, 0);

        	bool activatedMotor = config.getIntValue(XmlPath::Root / XmlPath::Motion / XmlPath::ActivateMotor);
            int itTimeMs = config.getIntValue(XmlPath::Root / XmlPath::Motion / XmlPath::IterationTimeMs);

           	legMotion_ptr->Init("config/input.txt", activatedMotor, true, 7000);
            //legMotion_ptr->InitWalk(pointD, startAngle, endAngle, activatedMotor, true, 7000);
            threadManager_ptr->attach(threadManager_ptr->create(90, [legMotion_ptr, itTimeMs]() mutable { legMotion_ptr->Run(itTimeMs); },
                                      ThreadManager::Task::LEGS_CONTROL)); 

        	/*
        	// Init Walk task
        	staticWalk.init("config/input.txt", false, true, true);
        	staticWalk.initPosition(7000);

        	threadManager_ptr->attach(threadManager_ptr->create(90, boost::bind(&StaticWalk::run, &staticWalk,
                                      config.getIntValue(XmlPath::Root / XmlPath::Motion / XmlPath::IterationTimeMs)),
                                      ThreadManager::Task::LEGS_CONTROL));
		     */
        }

        //threadManager_ptr->create(90, boost::bind(&MotorControl::run, &motorControl), ThreadManager::Task::MOTOR_CONTROL);
        //threadManager_ptr->timer(); // Start timer
        //boost_io.run();
        
        Logger::getInstance() << "END" << std::endl;
    }
    catch (std::exception& e)
    {
        Logger::getInstance(Logger::LogLvl::ERROR) << "Exception in main() : " << e.what() << std::endl;
    }
    return 0;
}
