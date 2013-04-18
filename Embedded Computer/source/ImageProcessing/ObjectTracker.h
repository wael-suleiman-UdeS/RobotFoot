#ifndef OBJECTTRACKER_H
#define OBJECTTRACKER_H

#include <opencv/cvaux.h>
#include "../Control/STM32F4.h"
#include "../Utilities/XmlParser.h"

//TODO :crap
#include <boost/filesystem.hpp>

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

	// TODO: remove hack
	void initializeHack(const XmlParser& config);

private:
	STM32F4* _controller;
	cv::Point _centerPosition;
	cv::Point _objectPosition;
	int _noObjectCount;
    static const int _noObjectMaxCount = 3;

	ObjectTracker() {}

	// todo: remove hack
	std::uint8_t _pan;
	std::uint8_t _tilt;
	std::uint8_t _horizontal;
	std::uint8_t _vertical;
	int _threshold;
};

#endif // OBJECTTRACKER_H