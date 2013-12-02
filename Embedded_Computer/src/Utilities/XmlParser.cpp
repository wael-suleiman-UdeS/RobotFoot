#include "XmlParser.h"

using boost::filesystem::path;
using pugi::xpath_node;
using pugi::xml_node;
using pugi::xpath_exception;

namespace XmlPath
{
	const char* Value = "value";

	const path Root = "RobotFoot";

	const path Sizes = "Sizes";
	const path ImageProcessing = "ImageProcessing";
	const path Camera = "Camera";
	const path Colors = "Colors";
	const path Color = "Color";
	const path HSVcolor = "HSVcolor";
	const path CircleSpec = "CircleSpec";

	const path BallColor = "BallColor";
	const path GoalColor = "GoalColor";

	const path Motion = "Motion";
	const path Motors = "Motors";
	const path ActivateMotor = "ActivateMotor";
    const path IterationTimeMs = "IterationTimeMs";
    const path Configurations = "Configurations";
	const path Head = "Head";
	const path Legs = "Legs";
    const path KP = "KP";
    const path KD = "KD";
    const path KI = "KI";

	const path Threshold = "Threshold";

	const path PermanentPelvisPitch = "COMPENSATION_OFFSET/PermanentPelvisPitch";
	const path RightPelvisPitchCompensationOffset = "COMPENSATION_OFFSET/RightPelvis/Pitch";
	const path RightPelvisRollCompensationOffset = "COMPENSATION_OFFSET/RightPelvis/Roll";
	const path RightPelvisYawCompensationOffset = "COMPENSATION_OFFSET/RightPelvis/Yaw";
	const path RightPelvisxCompensationOffset = "COMPENSATION_OFFSET/RightPelvis/x";
	const path RightPelvisyCompensationOffset = "COMPENSATION_OFFSET/RightPelvis/y";
	const path RightPelviszCompensationOffset = "COMPENSATION_OFFSET/RightPelvis/z";
	const path LeftPelvisPitchCompensationOffset = "COMPENSATION_OFFSET/LeftPelvis/Pitch";
	const path LeftPelvisRollCompensationOffset = "COMPENSATION_OFFSET/LeftPelvis/Roll";
	const path LeftPelvisYawCompensationOffset = "COMPENSATION_OFFSET/LeftPelvis/Yaw";
	const path LeftPelvisxCompensationOffset = "COMPENSATION_OFFSET/LeftPelvis/x";
	const path LeftPelvisyCompensationOffset = "COMPENSATION_OFFSET/LeftPelvis/y";
	const path LeftPelviszCompensationOffset = "COMPENSATION_OFFSET/LeftPelvis/z";
	const path RightFootPitchCompensationOffset = "COMPENSATION_OFFSET/RightFoot/Pitch";
	const path RightFootRollCompensationOffset = "COMPENSATION_OFFSET/RightFoot/Roll";
	const path RightFootYawCompensationOffset = "COMPENSATION_OFFSET/RightFoot/Yaw";
	const path RightFootxCompensationOffset = "COMPENSATION_OFFSET/RightFoot/x";
	const path RightFootyCompensationOffset = "COMPENSATION_OFFSET/RightFoot/y";
	const path RightFootzCompensationOffset = "COMPENSATION_OFFSET/RightFoot/z";
	const path LeftFootPitchCompensationOffset = "COMPENSATION_OFFSET/LeftFoot/Pitch";
	const path LeftFootRollCompensationOffset = "COMPENSATION_OFFSET/LeftFoot/Roll";
	const path LeftFootYawCompensationOffset = "COMPENSATION_OFFSET/LeftFoot/Yaw";
	const path LeftFootxCompensationOffset = "COMPENSATION_OFFSET/LeftFoot/x";
	const path LeftFootyCompensationOffset = "COMPENSATION_OFFSET/LeftFoot/y";
	const path LeftFootzCompensationOffset = "COMPENSATION_OFFSET/LeftFoot/z";


	const path UseCOM = "UseCOM";
	const path DISTANCETHRESHOLD = "DISTANCETHRESHOLD";
	const path ANGLETHRESHOLD = "ANGLETHRESHOLD";
	const path ITERATIONMAX = "ITERATIONMAX";
	const path StepTime = "StepTime";
	const path StepHeight = "StepHeight";
	const path StepLength = "StepLength";
	const path R_HIP_YAW = "R_HIP_YAW";
	const path L_HIP_YAW = "L_HIP_YAW";
	const path R_HIP_ROLL = "R_HIP_ROLL";
	const path L_HIP_ROLL = "L_HIP_ROLL";
	const path R_HIP_PITCH = "R_HIP_PITCH";
	const path L_HIP_PITCH = "L_HIP_PITCH";
	const path R_KNEE = "R_KNEE";
	const path L_KNEE = "L_KNEE";
	const path R_ANKLE_PITCH ="R_ANKLE_PITCH";
	const path L_ANKLE_PITCH = "L_ANKLE_PITCH";
	const path R_ANKLE_ROLL = "R_ANKLE_ROLL";
	const path L_ANKLE_ROLL = "L_ANKLE_ROLL";
	const path HEAD_PAN = "HEAD_PAN";
	const path HEAD_TILT = "HEAD_TILT";

	const path LegsMotors = Root / Motion / Motors / Legs;
	const path HeadMotors = Root / Motion / Motors / Head;
    const path MotorsConfig = Root / Motion / Configurations;

	const path MotorID = "MotorID";
	const path Offset = "Offset";
	const path LimitMin = "LimitMin";
	const path LimitMax = "LimitMax";
    const path PlayTime = "PlayTime";
    const path IsInversed = "IsInversed";
}

/** \brief Load an XML file for subsequent parsing operations
 *
 * \param filePath path: Path of the XML file to load
 * \return bool: Success of the loading
 *
 */
bool XmlParser::loadFile(path filePath)
{
	return _document.load_file(filePath.c_str());
}

/** \brief Retrieve a string value attribute in the XML document
 *
 * \param xPath path: XPath of the XML element in which to read the value attribute
 * \return string: Retrieved string
 *
 */
string XmlParser::getStringValue(path xPath) const
{
	if (_document.empty()) { return ""; }
	
	try
	{
		xpath_node node = _document.select_single_node(xPath.generic_string().c_str());
		return node.node().attribute(XmlPath::Value).value();
	}
	catch(const xpath_exception& ex) { return ""; }
}

/** \brief Retrieve the string value attribute of each child of the xPath Node
 *
 * \param xPath path: XPath of the XML element in which to read the value attributes
 * \return std::vector<string>: Vector of the retrieved values
 *
 */
std::vector<std::string> XmlParser::getChildrenStringValues(path xPath) const
{
    std::vector<std::string> values;
	if (_document.empty()) { return values; }
   
    try
    {
        xpath_node node = _document.select_single_node(xPath.generic_string().c_str());

        for (xml_node child = node.node().first_child(); child; child = child.next_sibling())
        {
            values.push_back(child.attribute(XmlPath::Value).value());
        }
        return values;
    } 
	catch(const xpath_exception& ex) { return std::vector<std::string>(); }
}

/** \brief Retrieve an int value attribute in the XML document
 *
 * \param xPath path: XPath of the XML element in which to read the value attribute
 * \return string: Retrieved int
 *
 */
float XmlParser::getIntValue(path xPath) const
{
	float result = 0;
	const auto s = getStringValue(xPath);
	if (1 != std::sscanf(s.c_str(), "%f", &result))
	{
		// It didn't work!!!
		result = 0;
	}
	return result;
}
