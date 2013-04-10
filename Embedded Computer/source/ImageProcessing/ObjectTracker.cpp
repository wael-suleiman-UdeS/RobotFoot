#include "ObjectTracker.h"


ObjectTracker::ObjectTracker()
{
    _objectPosition.x = -1;
    _objectPosition.y = -1;
    _noObjectCount = 0;
}


ObjectTracker::~ObjectTracker()
{
}

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
