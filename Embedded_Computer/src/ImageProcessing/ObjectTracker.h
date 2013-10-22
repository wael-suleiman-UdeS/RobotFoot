#ifndef OBJECTTRACKER_H
#define OBJECTTRACKER_H

#include <opencv/cvaux.h>
#include "Control/MotorControl.h"
#include "../Utilities/XmlParser.h"
#include "../Utilities/logger.h"
#include "../Utilities/PID.h"
#include <vector>

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
	ObjectTracker(MotorControl* mc, cv::Point center);
	~ObjectTracker() {}

	void track(cv::Point position);

	// TODO: remove hack
	void initializeHack(const XmlParser& config);
	void initializeHackPID(const XmlParser& config);

private:
	void scan();
	void readMotors();
	cv::Point limitAngle(cv::Point angle);

	MotorControl* _mc;
	cv::Point _centerPosition;
	cv::Point _objectError;
	int _noObjectCount;
	ObjectTracker() {}

	// todo: Values to be in a motor object
	cv::Point _currentAngle;
	std::uint8_t _panId;
	std::uint8_t _tiltId;
	int _threshold;
	int _minPan;
	int _maxPan;
	int _minTilt;
	int _maxTilt;
	unsigned int _scanningError;
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
