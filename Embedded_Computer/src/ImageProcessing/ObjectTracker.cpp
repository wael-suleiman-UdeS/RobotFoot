#include "ObjectTracker.h"

using cv::Point;
using boost::filesystem::path;

// todo: remove hack
void ObjectTracker::initializeHack(const XmlParser& config)
{
	// TODO: use values of MotorControl
	path headPath = XmlPath::Root / XmlPath::Motion / XmlPath::Motors / XmlPath::Head;

	_minPan = config.getIntValue(headPath / XmlPath::HEAD_PAN / XmlPath::LimitMin);
	_maxPan = config.getIntValue(headPath / XmlPath::HEAD_PAN / XmlPath::LimitMax);

	_minTilt = config.getIntValue(headPath / XmlPath::HEAD_TILT / XmlPath::LimitMin);
	_maxTilt = config.getIntValue(headPath / XmlPath::HEAD_TILT / XmlPath::LimitMax);

	_threshold = config.getIntValue(headPath / XmlPath::Threshold);

	_scanningError = config.getIntValue(headPath / "ScanningError");

	_noObjectMaxCount = config.getIntValue(headPath / "NoObjectMaxCount");

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
ObjectTracker::ObjectTracker(MotorControl* mc, Point center)
{
	_mc = mc;
	_objectError = Point(-1, -1);
	_currentAngle = Point(-1, -1);
	_newAngle = Point(-1, -1);
    _noObjectCount = 0;
	_centerPosition = center;
}

void ObjectTracker::track(Point objectPosition)
{

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
			if (_objectError.x > 0) { _objectError.x = -_scanningError; }
		}
	    Logger::getInstance() << "-------------------" << std::endl;
		return;
	}

	if (abs(_objectError.x) > _threshold)
	{
		_newAngle.x = _currentAngle.x + _pids["Pan"].process_PID(_objectError.x);
	}

	if (abs(_objectError.y) > _threshold)
	{
		_newAngle.y = _currentAngle.y + _pids["Tilt"].process_PID(_objectError.y);
	}

	setHeadAngles();

    Logger::getInstance() << "Object error: " << _objectError.x << ", " << _objectError.y << std::endl;
    Logger::getInstance() << "Current angle: " << _currentAngle.x << ", " << _currentAngle.y << std::endl;
    Logger::getInstance() << "New angle: " << _newAngle.x << ", " << _newAngle.y << std::endl;
    Logger::getInstance() << "-------------------" << std::endl;

}

void ObjectTracker::readHeadAngles() {
	std::vector<double> angles;
	_mc->ReadPositions(angles, MotorControl::Config::HEAD);
	_currentAngle.x = angles[0];
	_currentAngle.y = angles[1];
	_newAngle = _currentAngle;
}

void ObjectTracker::setHeadAngles() {
	std::vector<double> angles;
	angles.push_back(_newAngle.x);
	angles.push_back(_newAngle.y);
	_mc->SetPositions(angles, MotorControl::Config::HEAD);
}

void ObjectTracker::scan() {
	if (_currentAngle.x >= 0)
	{
	    if (_currentAngle.x < _minPan + _threshold) {
			_pids["Pan"].reset();
			_objectError.x = _scanningError;
		    Logger::getInstance() << "Start scanning to the right" << std::endl;
		} else if (_currentAngle.x > _maxPan - _threshold) {
			_pids["Pan"].reset();
			_objectError.x = -_scanningError;
		    Logger::getInstance() << "Start scanning to the left" << std::endl;
		}

	}

	if (_currentAngle.y >= 0)
	{
		_objectError.y = (_maxPan - _minPan) * 0.75 - _currentAngle.y;
	}
}

