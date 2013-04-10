#ifndef OBJECTTRACKER_H
#define OBJECTTRACKER_H

#include <opencv\cvaux.h>

class ObjectTracker
{
public:
	ObjectTracker();
	~ObjectTracker();

	void setCenter(CvPoint center);
	void track(CvPoint position);

private:
	CvPoint _centerPosition;
	CvPoint _objectPosition;
	int _noObjectCount;
    static const int _noObjectMaxCount = 15;
};

#endif // IMAGEPROCESSING_H
