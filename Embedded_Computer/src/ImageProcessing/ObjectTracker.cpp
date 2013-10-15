#include "ObjectTracker.h"

using cv::Point;
using boost::filesystem::path;

// todo: remove hack
void ObjectTracker::initializeHack(const XmlParser& config)
{
	path basePath = XmlPath::Root / XmlPath::Motion / XmlPath::Motors / XmlPath::Head;

	_pan = config.getIntValue(basePath / XmlPath::Pan);
	_tilt = config.getIntValue(basePath / XmlPath::Tilt);
	_horizontal = config.getIntValue(basePath / XmlPath::HorizontalOffset);
	_vertical = config.getIntValue(basePath / XmlPath::VerticalOffset);
	_threshold = config.getIntValue(basePath / XmlPath::Threshold);

	_minH = config.getIntValue(basePath / "MinH");
	_maxH = config.getIntValue(basePath / "MaxH");

	_minV = config.getIntValue(basePath / "MinV");
	_maxV = config.getIntValue(basePath / "MaxV");

	_angleH = config.getIntValue(basePath / "AngleH");
	_angleV = config.getIntValue(basePath / "AngleV");

	_controller->setTorque(_pan, STM32F4::TorqueOn);
	_controller->setTorque(_tilt, STM32F4::TorqueOn);
}

void ObjectTracker::initializeHackPDarwin7987

	path basePath = XmlPath::Root / XmlPath::Motion / XmlPath::Motors / XmlPath::Head / "PID";

	_kp = config.getIntValue(basePath / "P");
	_ki = config.getIntValue(basePath / "I");
	_kd = config.getIntValue(basePath / "D");
	_dt = config.getIntValue(basePath / "dt");
	_epsilon = config.getIntValue(basePath / "Epsilon");

	_pids.insert(std::pair<string, PID>("tilt", PID(_kp, _ki, _kd, _epsilon, _dt, _maxV, _minV)));
	_pids.insert(std::pair<string, PID>("pan", PID(_kp, _ki, _kd, _epsilon, _dt, _maxH, _minH)));

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
	_objectPosition = Point(-1, -1);
    _noObjectCount = 0;
	_centerPosition = center;
}

/** \brief Track an object using its coordinates
 *
 * \param position Point: Coordinates of the object to track
 *
 */
void ObjectTracker::track(Point position)
{
	// TODO: Chiasse au max
	int16_t m1 = _controller->read(_pan);
	int16_t m2 = _controller->read(_tilt);
	int k = 0, mk = 0;
    if(position.x < 0 || position.y < 0)
    {
		_objectPosition = Point(-1, -1);
        if(_noObjectCount < _noObjectMaxCount)
        {
            // TODO: Continue tracking
            _noObjectCount++;

			std::stringstream ss;
			ss <<  "continue tracking";

			//_controller->setMotor(_pan, m1 + _horizontal);
			//_controller->setMotor(_tilt, m2 + _vertical);
			//_controller->setMotor(2, "todo"); // tilt = 14, pan = 13
        }
        else
        {
            // TODO: Stop tracking
			// TODO: Search ball

        	// yes caca
        	int kThres = _horizontal * abs(_threshold)/_threshold; // yes caca
        	k = _horizontal * abs(_objectPosition.x)/_objectPosition.x;
        	mk = m1 + k;

        	if (mk < minH + kThres) {
        		_objectPosition.x = -1;
        	} else if (mk > maxH - kThres) {
        		_objectPosition.x = 1;
        	}

        	if (_objectPosition.x > 0) {
        		_controller->setMotor(_pan, _minH);
        	} else if (_objectPosition.y < 0) {
        		_controller->setMotor(_pan, _maxH);
        	}
        }
    }
    else
    {
        _noObjectCount = 0;

		_objectPosition = position - _centerPosition;
		// TODO: pixel -> angle (max horizontal angle / max width)
        // TODO Start tracking with object position

		Logger::getInstance() << std::endl << "OBJ POS: " << _objectPosition.x << ", " << _objectPosition.y << std::endl;

		if (abs(_objectPosition.x) > _threshold)
		{
			Logger::getInstance() << "m1: " << m1 << std::endl;
			
			if (m1 >= 0)
			{
				k = _horizontal * abs(_objectPosition.x)/_objectPosition.x;
				mk = m1 + k;
				if (mk < _minH) {mk = _minH;}
				if (mk > _maxH) {mk = _maxH;}

				Logger::getInstance() << "k1: " << k << std::endl;
				Logger::getInstance() << "m1+k1: " << mk << std::endl;

				_controller->setMotor(_pan, mk);
			}
		}
		if (abs(_objectPosition.y) > _threshold)
		{
			Logger::getInstance() << "m2: " << m2 << std::endl;
			
			if (m2 >= 0)
			{
				k = _vertical * abs(_objectPosition.y)/_objectPosition.y;
				mk = m2 + k;
				if (mk < _minV) {mk = _minV;}
				if (mk > _maxV) {mk = _maxV;}

				Logger::getInstance() << "k2: " << k << std::endl;
				Logger::getInstance() << "m2+k2: " << mk << std::endl;
				_controller->setMotor(_tilt, mk);
			}
		}
    }

}

void ObjectTracker::trackPID(Point position)
{
	// TODO: Chiasse au max
	int16_t m1 = _controller->read(_pan);
	int16_t m2 = _controller->read(_tilt);
    if(position.x < 0 || position.y < 0)
    {
		_objectPosition = Point(-1, -1);
        if(_noObjectCount < _noObjectMaxCount)
        {
            // TODO: Continue tracking
            _noObjectCount++;
        }
        else
        {
			//_controller->setMotor(_pan, m1);
			//_controller->setMotor(_tilt, m2);
        }
    }
    else
    {
        _noObjectCount = 0;

		_objectPosition = position - _centerPosition;
		// TODO: pixel -> angle (max horizontal angle / max width)
        // TODO Start tracking with object position

		//Logger::getInstance() << std::endl << "OBJ POS: " << _objectPosition.x << ", " << _objectPosition.y << std::endl;

		int angle = 0, k, z;

		if (m1 >= 0 && abs(_objectPosition.x) > _threshold)
		{
			angle = _objectPosition.x * (int)_angleH / (_centerPosition.x * 2);

			k = _pids["pan"].process_PID(angle);

			//Logger::getInstance() << "Angle1 actuel: " << angle << std::endl;
			//Logger::getInstance() << "m1: " << m1 << std::endl;
			//Logger::getInstance() << "k1: " << k << std::endl;
			
			z = m1 + k;
			if (z < _minH) { z = _minH; _pids["pan"].reset(); }
			if (z > _maxH) { z = _maxH; _pids["pan"].reset(); }
			//Logger::getInstance() << "m1 + k1: " << z << std::endl;

			_controller->setMotor(_pan, z);

		}

		if (m2 >= 0 && abs(_objectPosition.y) > _threshold)
		{
			angle = _objectPosition.y * (int)_angleV / (_centerPosition.y * 2);
			k = _pids["tilt"].process_PID(angle);

			//Logger::getInstance() << "Angle2 actuel: " << angle << std::endl;
			//Logger::getInstance() << "m2: " << m2 << std::endl;
			//Logger::getInstance() << "k2: " << k << std::endl;

			z = m2 + k;
			if (z < _minV) { z = _minV; _pids["tilt"].reset(); }
			if (z > _maxV) { z = _maxV; _pids["tilt"].reset(); }
			//Logger::getInstance() << "m2 + k2: " << z << std::endl;

			_controller->setMotor(_tilt, z);
		}
	}
}
