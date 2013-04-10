#include "ObjectTracker.h"

using namespace std;


ObjectTracker::ObjectTracker()
{
    _objectPosition.x = -1;
    _objectPosition.y = -1;
    _noObjectCount = 0;

}

ObjectTracker::~ObjectTracker()
{
}

void track(CvPoint position)
{
    if(position.x < 0 || position.y < 0)
    {
        _objectPosition.x = -1;
        _objectPosition.y = -1;
        if(_noObjectCount < _noObjectMaxCount)
        {
            // TODO: Continuer de bouger la tête
            _noObjectCount++;
        }
        else
        {
            // TODO: Objet perdu de vu, faire la procédure...
        }
    }
    else
    {
        _noObjectCount = 0;
        CvPoint center = new CvPoint(Camera::WIDTH/2, Camera::HEIGHT/2);
        CvPoint offset = position - center;
        offset *= -1;
        offset.x *= (Camera::VIEW_H_ANGLE / (double)Camera::WIDTH);
        offset.y *= (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT);
        _objectPosition = offset;
        //TODO Bouger la tête
    }
}
