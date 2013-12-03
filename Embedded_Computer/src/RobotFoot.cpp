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

    // Init IO_service for ThreadManager and MotorControl
    boost::asio::io_service boost_io;
    try
    {
        std::shared_ptr<ThreadManager> threadManager_ptr(new ThreadManager());
        std::shared_ptr<MotorControl> motorControl_ptr(new MotorControl(threadManager_ptr, config, boost_io));

        // Start io task
        threadManager_ptr->create(80, [&boost_io]() mutable { boost_io.run(); }, ThreadManager::Task::IO_CONTROL);

        // Wait for button start event
        while(!motorControl_ptr->GetButtonStatus(MotorControl::Button::BUTTON_3));

        bool isTracking = config.getIntValue(XmlPath::Root / XmlPath::ImageProcessing);
        bool isMoving   = config.getIntValue(XmlPath::Root / XmlPath::Motion);
        
        if (isTracking)
        {
            // Starting Head task
            HeadControlTask headControlTask(threadManager_ptr, config, motorControl_ptr);
            threadManager_ptr->create(50, [headControlTask]() mutable { headControlTask.run(); }, ThreadManager::Task::HEAD_CONTROL);
        }

        std::shared_ptr<LegMotion> legMotion(new LegMotion(threadManager_ptr, motorControl_ptr, config));
        bool activatedMotor = config.getIntValue(XmlPath::Root / XmlPath::Motion / XmlPath::ActivateMotor);
        int itTimeMs = config.getIntValue(XmlPath::Root / XmlPath::Motion / XmlPath::IterationTimeMs);
        
        Eigen::Vector2f pointD;
        Eigen::Vector2f startAngle;
        Eigen::Vector2f endAngle;
        while (1) // main loop
        {
            if (isMoving)
            {
                if (isTracking)
                {
                    motorControl_ptr->ResetObjectDistance();
                    while(motorControl_ptr->GetObjectDistance().x == 0);
                    
                    pointD = Eigen::Vector2f(motorControl_ptr->GetObjectDistance().x, motorControl_ptr->GetObjectDistance().y);
                    startAngle = Eigen::Vector2f(0, 0);
                    endAngle = Eigen::Vector2f(0, 0);
                }
                else
                {
                    pointD = Eigen::Vector2f(0.2, 0);
                    startAngle = Eigen::Vector2f(0, 0);
                    endAngle = Eigen::Vector2f(0, 0);
                }
                // Start Motion task
                // legMotion->Init("config/input.txt", activatedMotor, true, 3000);
                legMotion->InitWalk(pointD, startAngle, endAngle, activatedMotor, true, 3000);
                threadManager_ptr->attach(threadManager_ptr->create(90, [legMotion, itTimeMs]() mutable { legMotion->Run(itTimeMs); }, ThreadManager::Task::LEGS_CONTROL)); 
                //legMotion->InitKick(activatedMotor, true, 3000, 2.0);
                //threadManager_ptr->attach(threadManager_ptr->create(90, [legMotion, itTimeMs]() mutable { legMotion->Run(itTimeMs); }, ThreadManager::Task::LEGS_CONTROL)); 
            while(1)
                Logger::getInstance() << "END WALK" << std::endl;
            }
            else
            {
                threadManager_ptr->attach(ThreadManager::Task::IO_CONTROL);
            }
        }
        Logger::getInstance() << "END" << std::endl;
    }
    catch (std::exception& e)
    {
        Logger::getInstance(Logger::LogLvl::ERROR) << "Exception in main() : " << e.what() << std::endl;
    }
    return 0;
}
