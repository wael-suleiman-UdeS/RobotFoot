#include "ObjectTracker.h"

using cv::Point;
using boost::filesystem::path;

// todo: remove hack
void ObjectTracker::initializeHack(const XmlParser& config)
{
	// TODO: use values of MotorControl
	path headPath = XmlPath::Root / XmlPath::Motion / XmlPath::Motors / XmlPath::Head;

	std::vector<double> minAngles;
	std::vector<double> maxAngles;
 	_mc->HardGetMinAngles(minAngles, MotorControl::Config::HEAD);
	_mc->HardGetMaxAngles(maxAngles, MotorControl::Config::HEAD);

	if (minAngles.size() >= 2 && maxAngles.size() >= 2)
	{
		_minPan = minAngles[0];
		_maxPan = maxAngles[0];

		_minTilt = minAngles[1];
		_maxTilt = maxAngles[1];
	}

	_threshold = config.getIntValue(headPath / XmlPath::Threshold);

	_scanningError = config.getIntValue(headPath / "ScanningError");

	_noObjectMaxCount = config.getIntValue(headPath / "NoObjectMaxCount");

    _robotHeight = config.getIntValue(XmlPath::Root / XmlPath::Sizes / "RobotHeight");

	_mc->SetTorque(true, MotorControl::Config::HEAD);
}

void ObjectTracker::initializeHackPID(const XmlParser& config) {
	// TODO: put PID outside of this class
	path basePath = XmlPath::Root / XmlPath::Motion / XmlPath::Motors / XmlPath::Head / "PID";

	_kp = config.getIntValue(basePath / "P");
	_ki = config.getIntValue(basePath / "I");
	_kd = config.getIntValue(basePath / "D");
	_dt = config.getIntValue(basePath / "dt");
	_epsilon = config.getIntValue(basePath / "Epsilon");

	_pids.insert(std::pair<string, PID>("Pan", PID(_kp, _ki, _kd, _epsilon, _dt, _maxPan, _minPan)));
	_pids.insert(std::pair<string, PID>("Tilt", PID(_kp, _ki, _kd, _epsilon, _dt, _maxTilt, _minTilt)));

}

/** \brief Constructor
 *
 * \param serial USBInterface*: Serial Interface for communication with the control board
 * \param center Point: Center of the camera used for calibration
 *
 */
ObjectTracker::ObjectTracker(std::shared_ptr<ThreadManager> threadManager_ptr, std::shared_ptr<MotorControl> mc_ptr, Point center)
:
    _threadManager(threadManager_ptr),
    _mc(mc_ptr)
{
	_objectError = Point(-1, -1);
	_currentAngle = Point(-1, -1);
	_newAngle = Point(-1, -1);
    _noObjectCount = 0;
	_centerPosition = center;
}

void ObjectTracker::track(Point objectPosition)
{
	Logger::getInstance() << "Object distance: " << _mc->GetObjectPosition() << " cm" << std::endl;
    Logger::getInstance() << "Object position: " << objectPosition.x << ", " << objectPosition.y << std::endl;
    Logger::getInstance() << "No object count: " << _noObjectCount << "/" << _noObjectMaxCount << std::endl;

    readHeadAngles();

    if ( objectPosition.x > 0 && objectPosition.y > 0) {
		_objectError = _centerPosition - objectPosition;
		_noObjectCount = 0;
    }
    else if(_noObjectCount > _noObjectMaxCount) {
		scan();
	}
	else {
		_noObjectCount++;

		if (_noObjectCount > _noObjectMaxCount) {
			_pids["Pan"].reset();
			if (_objectError.x <= 0) { _objectError.x = _scanningError; }
			else if (_objectError.x > 0) { _objectError.x = -_scanningError; }
		}
	    Logger::getInstance() << "-------------------" << std::endl;
		return;
	}

	if (abs(_objectError.x) > _threshold)
	{
		_newAngle.x = _currentAngle.x + _pids["Pan"].process_PID(_objectError.x);
	}

	if(_noObjectCount > _noObjectMaxCount) {} // todo: remove hack
	else if (abs(_objectError.y) > _threshold)
	{
		_newAngle.y = _currentAngle.y + _pids["Tilt"].process_PID(_objectError.y);
	}

	if (_newAngle.x < _minPan || _newAngle.x > _maxPan) { _pids["Pan"].reset(); }
	if (_newAngle.y < _minTilt || _newAngle.y > _maxTilt) { _pids["Tilt"].reset(); }

	if (abs(_objectError.x) > _threshold && abs(_objectError.y) > _threshold)
	{
		processObjectDistance();
	}

    Logger::getInstance() << "Object error: " << _objectError.x << ", " << _objectError.y << std::endl;
    Logger::getInstance() << "Current angle: " << _currentAngle.x << ", " << _currentAngle.y << std::endl;
    Logger::getInstance() << "New angle: " << _newAngle.x << ", " << _newAngle.y << std::endl;
    Logger::getInstance() << "-------------------" << std::endl;

}

void ObjectTracker::readHeadAngles() {
	std::vector<double> angles;
	_mc->ReadPositions(angles, MotorControl::Config::HEAD);
	//_mc->HardGet(angles, MotorControl::Config::HEAD);

	if (angles.size() >= 2)
	{
		_currentAngle.x = angles[0];
		_currentAngle.y = angles[1];
	}
	_newAngle = _currentAngle;
}

void ObjectTracker::setHeadAngles() {
	std::vector<double> angles;
	angles.push_back(_newAngle.x);
	angles.push_back(_newAngle.y);

	_mc->SetPositions(angles, MotorControl::Config::HEAD);
	//_mc->HardSet(angles, MotorControl::Config::HEAD);
}

void ObjectTracker::scan() {

	Logger::getInstance() << "_minPan + _threshold: " << _minPan + _threshold << std::endl;
	Logger::getInstance() << "_maxPan + _threshold: " << _maxPan + _threshold << std::endl;


	if (_currentAngle.x < _minPan + _threshold) {
		_pids["Pan"].reset();
		_objectError.x = _scanningError;
		_newAngle.y = -15; // todo: remove hack
		Logger::getInstance() << "Start scanning to the right" << std::endl;
	} else if (_currentAngle.x > _maxPan - _threshold) {
		_pids["Pan"].reset();
		_objectError.x = -_scanningError;
		_newAngle.y = -90; // todo: remove hack
		Logger::getInstance() << "Start scanning to the left" << std::endl;
	}

}

void ObjectTracker::processObjectDistance()
{
	cv::Point radianAngle;

	radianAngle.x = _currentAngle.x * M_PI/180;
	radianAngle.y = std::abs(_currentAngle.y * M_PI/180);

	double euclidianDistance = _robotHeight * std::tan((M_PI/2)-radianAngle.y);
	ObjectPosition objectDistance;
	objectDistance.x = euclidianDistance * std::sin(radianAngle.x);
	objectDistance.y = euclidianDistance * std::cos(radianAngle.x);

	Logger::getInstance() << "Euclidian distance: " << euclidianDistance << " cm" << std::endl;

	_mc->SetObjectDistance(objectDistance);
}
