#include "ObjectTracker.h"

/** \brief Constructor
 *
 * \param center CvPoint: Center of the camera used for calibration
 *
 */
ObjectTracker::ObjectTracker(CvPoint center)
{
    _objectPosition.x = -1;
    _objectPosition.y = -1;
    _noObjectCount = 0;
	_centerPosition = center;
}

/** \brief Track an object using its coordinates
 *
 * \param position CvPoint: Coordinates of the object to track
 *
 */
void ObjectTracker::track(CvPoint position)
{
    if(position.x < 0 || position.y < 0)
    {
        _objectPosition.x = -1;
        _objectPosition.y = -1;
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
        _objectPosition.x = (position.x - _centerPosition.x) * -1;
		_objectPosition.y = (position.y - _centerPosition.y) * -1;
		// TODO: pixel -> angle (max horizontal angle / max width)
        // TODO Start tracking with object position
    }
}
