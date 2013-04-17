#ifndef OBJECTTRACKER_H
#define OBJECTTRACKER_H

#include <opencv/cvaux.h>
#include "../Control/STM32F4.h"

/** @addtogroup Image Processing
 * @{
 */

/** \brief Class for tracking an object position
 */
class ObjectTracker
{
public:
	ObjectTracker(STM32F4* controller, cv::Point center);
	~ObjectTracker() {}

	void track(cv::Point position);

private:
	STM32F4* _controller;
	cv::Point _centerPosition;
	cv::Point _objectPosition;
	int _noObjectCount;
    static const int _noObjectMaxCount = 15;

	ObjectTracker() {}
};

#endif // OBJECTTRACKER_H