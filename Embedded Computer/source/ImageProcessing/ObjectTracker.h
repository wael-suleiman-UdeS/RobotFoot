#ifndef OBJECTTRACKER_H
#define OBJECTTRACKER_H

#include <opencv\cvaux.h>

/** @addtogroup Image Processing
 * @{
 */

/** \brief Class for tracking an object position
 */
class ObjectTracker
{
public:
	ObjectTracker(CvPoint center);
	~ObjectTracker() {}

	void track(CvPoint position);

private:
	CvPoint _centerPosition;
	CvPoint _objectPosition;
	int _noObjectCount;
    static const int _noObjectMaxCount = 15;

	ObjectTracker() {}
};

#endif // IMAGEPROCESSING_H
