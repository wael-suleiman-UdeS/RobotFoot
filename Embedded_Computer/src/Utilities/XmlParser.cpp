#include "XmlParser.h"

using boost::filesystem::path;
using pugi::xpath_node;
using pugi::xml_node;
using pugi::xpath_exception;

namespace XmlPath
{
	const char* Value = "value";

	const path Root = "RobotFoot";

	const path ImageProcessing = "ImageProcessing";
	const path Camera = "Camera";
	const path Colors = "Colors";
	const path Color = "Color";
	const path HSVcolor = "HSVcolor";
	const path CircleSpec = "CircleSpec";

	const path Motion = "Motion";
	const path Motors = "Motors";
	const path Head = "Head";
	const path Legs = "Legs";
	const fs::path Pan = "Pan";
	const fs::path Tilt = "Tilt";
	const fs::path HorizontalOffset = "HorizontalOffset";
	const fs::path VerticalOffset = "VerticalOffset";
	const fs::path Threshold = "Threshold";

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

	const path LegsMotors = Root / Motion / Motors / Legs;

	const path MotorID = "MotorID";
	const path Offset = "Offset";
	const path LimitMin = "LimitMin";
	const path LimitMax = "LimitMax";
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
std::vector<std::string> XmlParser::getStringValues(path xPath) const
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
