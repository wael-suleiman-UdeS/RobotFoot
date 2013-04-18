#include "ObjectTracker.h"

using cv::Point;
using boost::filesystem::path;

// todo: remove hack
void ObjectTracker::initializeHack(const XmlParser& config)
{
	path basePath = XmlPath::Root / XmlPath::Motion / XmlPath::Head;

	_pan = config.getIntValue(basePath / XmlPath::Pan);
	_tilt = config.getIntValue(basePath / XmlPath::Tilt);
	_horizontal = config.getIntValue(basePath / XmlPath::HorizontalOffset);
	_vertical = config.getIntValue(basePath / XmlPath::VerticalOffset);
	_threshold = config.getIntValue(basePath / XmlPath::Threshold);
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
	uint16_t m1 = _controller->read(_pan);
	uint16_t m2 = _controller->read(_tilt);

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
			//_controller->setMotor(_pan, m1);
			//_controller->setMotor(_tilt, m2);
        }
    }
    else
    {
        _noObjectCount = 0;

		_objectPosition = (position - _centerPosition) * -1;
		// TODO: pixel -> angle (max horizontal angle / max width)
        // TODO Start tracking with object position

		if (abs(_objectPosition.x) > _threshold)
		{
			_controller->setMotor(_pan, m1 + (_horizontal * abs(_objectPosition.x)/_objectPosition.x));
			_controller->setMotor(_tilt, m2 + (_vertical * abs(_objectPosition.y)/_objectPosition.y));
		}
    }

}