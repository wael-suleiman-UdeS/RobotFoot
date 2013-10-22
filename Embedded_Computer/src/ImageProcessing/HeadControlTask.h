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

#include "ImageProcessing/ColorFinder.h"
#include "ImageProcessing/ObjectTracker.h"
#include "Utilities/XmlParser.h"
#include "Utilities/ThreadManager.h"
#include "Control/MotorControl.h"

class HeadControlTask
{
public:
   HeadControlTask(ThreadManager *threadManager, XmlParser &config, MotorControl &mc);
   ~HeadControlTask();

   void start();

private:
    ThreadManager *_threadManager;
	cv::Point _ballPosition;
	CircleSpec *_circle;
	ColorFinder *_finder;
	ObjectTracker *_tracker;
	bool _guiEnabled;
	double _durationMean;
	int _durationIndex;
};

#endif  //HEAD_CONTROL_TASK_H
