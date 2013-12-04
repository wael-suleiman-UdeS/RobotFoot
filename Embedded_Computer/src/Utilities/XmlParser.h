#ifndef XMLPARSER_H
#define XMLPARSER_H

#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include "pugixml.h"

using std::string;
namespace fs = boost::filesystem;

/** @addtogroup Utilities
 * @{
 */

inline fs::path operator+(fs::path p1, fs::path p2)
{
	return p1 += p2;
}

/** \brief Namespace containing the XPaths of the elements in the XML configuration file
 */
namespace XmlPath
{
	extern const char* Value;

	extern const fs::path Root;

	extern const fs::path Sizes;
	extern const fs::path ImageProcessing;
	extern const fs::path Camera;
	extern const fs::path Colors;
	extern const fs::path Color;
	extern const fs::path HSVcolor;
	extern const fs::path CircleSpec;

	extern const fs::path Objects;

	extern const fs::path Motion;
	extern const fs::path Motors;
	extern const fs::path ActivateMotor;
    extern const fs::path IterationTimeMs;
    extern const fs::path Configurations;
	extern const fs::path Head;
	extern const fs::path Threshold;

    extern const fs::path KP;
    extern const fs::path KD;
    extern const fs::path KI;

    extern const fs::path PerformInitPosition;
    extern const fs::path PermanentPelvisPitch;
    extern const fs::path RightPelvisPitchCompensationOffset;
    extern const fs::path RightPelvisRollCompensationOffset;
    extern const fs::path RightPelvisYawCompensationOffset;
    extern const fs::path RightPelvisxCompensationOffset;
    extern const fs::path RightPelvisyCompensationOffset;
    extern const fs::path RightPelviszCompensationOffset;
    extern const fs::path LeftPelvisPitchCompensationOffset;
    extern const fs::path LeftPelvisRollCompensationOffset;
    extern const fs::path LeftPelvisYawCompensationOffset;
    extern const fs::path LeftPelvisxCompensationOffset;
    extern const fs::path LeftPelvisyCompensationOffset;
    extern const fs::path LeftPelviszCompensationOffset;
    extern const fs::path RightFootPitchCompensationOffset;
    extern const fs::path RightFootRollCompensationOffset;
    extern const fs::path RightFootYawCompensationOffset;
    extern const fs::path RightFootxCompensationOffset;
    extern const fs::path RightFootyCompensationOffset;
    extern const fs::path RightFootzCompensationOffset;
    extern const fs::path LeftFootPitchCompensationOffset;
    extern const fs::path LeftFootRollCompensationOffset;
    extern const fs::path LeftFootYawCompensationOffset;
    extern const fs::path LeftFootxCompensationOffset;
    extern const fs::path LeftFootyCompensationOffset;
    extern const fs::path LeftFootzCompensationOffset;

    extern const fs::path UseCOM;
    extern const fs::path DISTANCETHRESHOLD;
    extern const fs::path ANGLETHRESHOLD;
    extern const fs::path MaxPosError;
    extern const fs::path ITERATIONMAX;
    extern const fs::path StepTime;
    extern const fs::path StepHeight;
    extern const fs::path StepLength;
	extern const fs::path R_HIP_YAW;
	extern const fs::path L_HIP_YAW;
	extern const fs::path R_HIP_ROLL;
	extern const fs::path L_HIP_ROLL;
	extern const fs::path R_HIP_PITCH;
	extern const fs::path L_HIP_PITCH;
	extern const fs::path R_KNEE;
	extern const fs::path L_KNEE;
	extern const fs::path R_ANKLE_PITCH;
	extern const fs::path L_ANKLE_PITCH;
	extern const fs::path R_ANKLE_ROLL;
	extern const fs::path L_ANKLE_ROLL;
	extern const fs::path HEAD_PAN;
	extern const fs::path HEAD_TILT;

	extern const fs::path LegsMotors;
	extern const fs::path HeadMotors;
    extern const fs::path MotorsConfig;

	extern const fs::path MotorID;
	extern const fs::path Offset;
	extern const fs::path LimitMin;
	extern const fs::path LimitMax;
    extern const fs::path PlayTime;
    extern const fs::path IsInversed;
}

/** @brief Class for XML parsing
 */
class XmlParser
{
private:
	pugi::xml_document _document; /**< XML document to parse */

public:
	XmlParser() {}
	~XmlParser() {}

	bool loadFile(fs::path filePath);
	string getStringValue(fs::path xPath) const;
    std::vector<std::string> getChildrenStringValues(fs::path xPath) const;
	float getIntValue(fs::path xPath) const;
};

#endif // XMLPARSER_H
