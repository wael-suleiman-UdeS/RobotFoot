#ifndef OBJECTTRACKER_H
#define OBJECTTRACKER_H

#include <opencv\cvaux.h>
#include "..\Utilities\USBInterface.h"

/** @addtogroup Image Processing
 * @{
 */

/** \brief Class for tracking an object position
 */
class ObjectTracker
{
public:
	ObjectTracker(USBInterface* serial, cv::Point center);
	~ObjectTracker() {}

	void track(cv::Point position);

private:
	USBInterface* _serial;
	cv::Point _centerPosition;
	cv::Point _objectPosition;
	int _noObjectCount;
    static const int _noObjectMaxCount = 15;

	ObjectTracker() {}
};

#endif // IMAGEPROCESSING_H
