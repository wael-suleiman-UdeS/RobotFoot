#ifndef HEAD_CONTROL_TASK_H
#define HEAD_CONTROL_TASK_H
/*!
 * \file HeadControlTask.h
 * \brief Track the ball
 * 
 * A class for the tracking of the ball
 * 
 * \authors David Dumont and Mitchel Labonté
 * \version 0.1
 */
#include <iostream>
#include <memory> // shared_ptr

#include "ImageProcessing/ColorFinder.h"
#include "ImageProcessing/ObjectTracker.h"
#include "Utilities/XmlParser.h"
#include "Utilities/ThreadManager.h"
#include "Control/MotorControl.h"

class HeadControlTask
{
public:
   HeadControlTask(std::shared_ptr<ThreadManager> threadManager_ptr, XmlParser &config, std::shared_ptr<MotorControl> mc_ptr);
   ~HeadControlTask();

   void run();

private:
    std::shared_ptr<ThreadManager> _threadManager;
	cv::Point _ballPosition;
    std::shared_ptr <CircleSpec> _circle;
    std::shared_ptr <ColorFinder> _finder;
    std::shared_ptr <ObjectTracker> _tracker;
	bool _guiEnabled;
	double _durationMean;
	int _durationIndex;
};

#endif  //HEAD_CONTROL_TASK_H
