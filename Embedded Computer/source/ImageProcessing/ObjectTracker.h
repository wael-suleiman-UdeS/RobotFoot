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
	ObjectTracker(cv::Point center);
	~ObjectTracker() {}

	void track(cv::Point position);

private:
	cv::Point _centerPosition;
	cv::Point _objectPosition;
	int _noObjectCount;
    static const int _noObjectMaxCount = 15;

	ObjectTracker() {}
};

#endif // IMAGEPROCESSING_H
