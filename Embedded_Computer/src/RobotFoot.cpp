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
        std::shared_ptr<HeadControlTask> headControlTask(new HeadControlTask(threadManager_ptr, config, motorControl_ptr));
        std::shared_ptr<LegMotion> legMotion(new LegMotion(threadManager_ptr, motorControl_ptr, config));
        
        bool isTracking = config.getIntValue(XmlPath::Root / XmlPath::ImageProcessing);
        bool isMoving   = config.getIntValue(XmlPath::Root / XmlPath::Motion);
        bool activatedMotor = config.getIntValue(XmlPath::Root / XmlPath::Motion / XmlPath::ActivateMotor);
        int itTimeMs = config.getIntValue(XmlPath::Root / XmlPath::Motion / XmlPath::IterationTimeMs);
        
        ObjectPosition object;
        Eigen::Vector2f pointD;
        Eigen::Vector2f startAngle;
        Eigen::Vector2f endAngle;

        // Start io task
        threadManager_ptr->create(80, [&boost_io]() mutable { boost_io.run(); }, ThreadManager::Task::IO_CONTROL);
        while (1)
        {
            // Wait for button start event
            while(motorControl_ptr->isPaused());

            if (isTracking)
            {
                // Starting Head task
                threadManager_ptr->create(50, [headControlTask]() mutable { headControlTask->run(); }, ThreadManager::Task::HEAD_CONTROL);
            }

            if (isMoving)
            {
                legMotion->InitPosition(3000);
            }
            while (!motorControl_ptr->isPaused()) // main loop
            {
                if (isMoving)
                {
                    if (isTracking)
                    {
                        motorControl_ptr->ResetObjectDistance();
                        while(motorControl_ptr->GetObjectDistance().x == 0);
                        object = motorControl_ptr->GetObjectDistance();
                    }
                    else
                    {
                        // Dummy object detected
                        object.x = 0.1;
                        object.y = 0;
                        object.angle = 0;
                    }
                    pointD = Eigen::Vector2f(object.x, object.y);
                    startAngle = Eigen::Vector2f(0, 0);
                    endAngle = Eigen::Vector2f(0, 0);

                    // Choose kick or walk and start motion task
                    if (object.x <= 0.05)
                    {
                        legMotion->InitKick(activatedMotor, true, 2.0);
                    }
                    else
                    {
                        legMotion->InitWalk(pointD, startAngle, endAngle, activatedMotor, true);
                    }                    
                    threadManager_ptr->create(90, [legMotion, itTimeMs]() mutable { legMotion->Run(itTimeMs); }, ThreadManager::Task::LEGS_CONTROL);
                    threadManager_ptr->attach(ThreadManager::Task::LEGS_CONTROL); 
                }
                else
                {
                    threadManager_ptr->attach(ThreadManager::Task::HEAD_CONTROL);
                }
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
