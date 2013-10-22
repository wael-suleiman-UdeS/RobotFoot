#include "ObjectTracker.h"

using cv::Point;
using boost::filesystem::path;

// todo: remove hack
void ObjectTracker::initializeHack(const XmlParser& config)
{
	path headPath = XmlPath::Root / XmlPath::Motion / XmlPath::Motors / XmlPath::Head;

	_panId = config.getIntValue(headPath / XmlPath::Pan / XmlPath::MotorID);
	_minPan = config.getIntValue(headPath / XmlPath::Pan / XmlPath::LimitMin);
	_maxPan = config.getIntValue(headPath / XmlPath::Pan / XmlPath::LimitMax);

	_tiltId = config.getIntValue(headPath / XmlPath::Tilt / XmlPath::MotorID);
	_minTilt = config.getIntValue(headPath / XmlPath::Tilt / XmlPath::LimitMin);
	_maxTilt = config.getIntValue(headPath / XmlPath::Tilt / XmlPath::LimitMax);

	_threshold = config.getIntValue(headPath / XmlPath::Threshold);

	_scanningError = config.getIntValue(headPath / "ScanningError");

	_noObjectMaxCount = config.getIntValue(headPath / "NoObjectMaxCount");

	_controller->setTorque(_panId, STM32F4::TorqueOn);
	_controller->setTorque(_tiltId, STM32F4::TorqueOn);
}

void ObjectTracker::initializeHackPID(const XmlParser& config) {

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
ObjectTracker::ObjectTracker(STM32F4* controller, Point center)
{
	_controller = controller;
	_objectError = Point(-1, -1);
    _noObjectCount = 0;
	_centerPosition = center;
}

void ObjectTracker::track(Point position)
{

    Logger::getInstance() << "Position: " << position.x << ", " << position.y << std::endl;
    Logger::getInstance() << "No object count: " << _noObjectCount << "/" << _noObjectMaxCount << std::endl;

    readMotors(); // TODO: this is for now

    if ( position.x > 0 && position.y > 0) {
		_objectError = _centerPosition - position;
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

	Point newAngle(-1,-1);

	if (_currentAngle.x >= 0 && abs(_objectError.x) > _threshold)
	{
		newAngle.x = _currentAngle.x + _pids["Pan"].process_PID(_objectError.x);
	}

	if (_currentAngle.y >= 0 && abs(_objectError.y) > _threshold)
	{
		newAngle.y = _currentAngle.y + _pids["Tilt"].process_PID(_objectError.y);
	}

	limitAngle(newAngle);
	if (newAngle.x > 0) { _controller->setMotor(_panId, newAngle.x); }
	if (newAngle.y > 0) { _controller->setMotor(_tiltId, newAngle.y); }

    Logger::getInstance() << "Object error: " << _objectError.x << ", " << _objectError.y << std::endl;
    Logger::getInstance() << "Current angle: " << _currentAngle.x << ", " << _currentAngle.y << std::endl;
    Logger::getInstance() << "New angle: " << newAngle.x << ", " << newAngle.y << std::endl;
    Logger::getInstance() << "-------------------" << std::endl;

}

void ObjectTracker::readMotors() {
	// If I don't put data in int16, I get high values instead of negative values
	int16_t x = _controller->read(_panId);
	int16_t y = _controller->read(_tiltId);
    Logger::getInstance() << "Current angle (16bits): " << x << ", " << y << std::endl;
	_currentAngle.x = x;
	_currentAngle.y = y;
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

Point ObjectTracker::limitAngle(Point angle) {
	if (angle.x < _minPan) { angle.x = _minPan; _pids["Pan"].reset(); }
	else if (angle.x > _maxPan) { angle.x = _maxPan; _pids["Pan"].reset(); }

	if (angle.y < _minTilt) { angle.y = _minTilt; _pids["Tilt"].reset(); }
	else if (angle.y > _maxTilt) { angle.y = _maxTilt; _pids["Tilt"].reset(); }

	return angle;
}

