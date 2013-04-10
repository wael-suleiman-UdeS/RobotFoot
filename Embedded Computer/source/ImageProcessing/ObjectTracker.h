#ifdef _OBJECT_TRACKER_H_
#define _OBJECT_TRACKER_H_

#include <iostream>
#include "ImageProcessing.h"

class ObjectTracker
{

private:
    int _noObjectCount;
    static const int _noObjectMaxCount = 15;

public:
    CvPoint _objectPosition;

    ObjectTracker();
    ~ObjectTracker();

    void track(CvPoint position);

};

#endif
