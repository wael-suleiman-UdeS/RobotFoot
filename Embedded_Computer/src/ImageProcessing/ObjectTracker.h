#ifndef OBJECTTRACKER_H
#define OBJECTTRACKER_H

#include <opencv/cvaux.h>
#include "../Control/STM32F4.h"
#include "../Utilities/XmlParser.h"
#include "../Utilities/logger.h"
#include "../Utilities/PID.h"

//TODO :crap
#include <boost/filesystem.hpp>
#include <map>

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
	void trackPID(cv::Point position);

	// TODO: remove hack
	void initializeHack(const XmlParser& config);
	void initializeHackPID(const XmlParser& config);

private:
	STM32F4* _controller;
	cv::Point _centerPosition;
	cv::Point _objectPosition;
	int _noObjectCount;
	ObjectTracker() {}

	// todo: remove hack
	std::uint8_t _pan;
	std::uint8_t _tilt;
	int _horizontal;
	int _vertical;
	int _threshold;
	int _minH;
	int _maxH;
	int _minV;
	int _maxV;
	unsigned int _angleH;
	unsigned _angleV;
    int _noObjectMaxCount;

	//todo: pid values elsewhere
	float _kp;
	float _ki;
	float _kd;
	float _dt;
	float _epsilon;
	std::map<std::string, PID> _pids;
};

#endif // OBJECTTRACKER_H
