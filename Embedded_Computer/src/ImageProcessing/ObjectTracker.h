#ifndef OBJECTTRACKER_H
#define OBJECTTRACKER_H

#include "Control/MotorControl_2.h"
#include "Utilities/XmlParser.h"
#include "Utilities/logger.h"
#include "Utilities/PID.h"
#include "Utilities/ThreadManager.h"

#include <vector>
#include <memory> // shared_ptr
#include <opencv/cvaux.h>

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
	ObjectTracker(std::shared_ptr<ThreadManager> threadManager_ptr, std::shared_ptr<MotorControl> mc_ptr, cv::Point center);
	~ObjectTracker() {}

	void track(cv::Point objectPosition);

	// TODO: remove hack
	void initializeHack(const XmlParser& config);
	void initializeHackPID(const XmlParser& config);

private:
	void scan();
	void readHeadAngles();
	void setHeadAngles();
	void processObjectDistance();

    std::shared_ptr<ThreadManager> _threadManager;
    std::shared_ptr<MotorControl> _mc;
	cv::Point _centerPosition;
	cv::Point _objectError;
	int _noObjectCount;

	// todo: Values to be in a motor object
	cv::Point _currentAngle;
	cv::Point _newAngle;
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

	// todo: Should I not know that?
	double _robotHeight;
};

#endif // OBJECTTRACKER_H
