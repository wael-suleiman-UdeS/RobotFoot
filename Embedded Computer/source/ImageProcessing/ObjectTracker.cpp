#include "ObjectTracker.h"

using cv::Point;

/** \brief Constructor
 *
 * \param center Point: Center of the camera used for calibration
 *
 */
ObjectTracker::ObjectTracker(Point center)
{
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
        }
        else
        {
            // TODO: Stop tracking
			// TODO: Search ball
        }
    }
    else
    {
        _noObjectCount = 0;

		_objectPosition = (position - _centerPosition) * -1;
		// TODO: pixel -> angle (max horizontal angle / max width)
        // TODO Start tracking with object position
    }
}
