#include "ObjectTracker.h"

using cv::Point;

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
    if(position.x < 0 || position.y < 0)
    {
		_objectPosition = Point(-1, -1);
        if(_noObjectCount < _noObjectMaxCount)
        {
            // TODO: Continue tracking
            _noObjectCount++;

			std::stringstream ss;
			ss <<  "continue tracking";
			_controller->setMotor('\xfd', 25);
			//_controller->setMotor(2, "todo"); // tilt = 14, pan = 13
        }
        else
        {
            // TODO: Stop tracking
			// TODO: Search ball
			_controller->setMotor('\xfd', 300);
        }
    }
    else
    {
        _noObjectCount = 0;

		_objectPosition = (position - _centerPosition) * -1;
		// TODO: pixel -> angle (max horizontal angle / max width)
        // TODO Start tracking with object position

		_controller->setMotor('\xfd', 25);
    }
}