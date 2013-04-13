#include "XmlParser.h"

using boost::filesystem::path;
using pugi::xpath_node;
using pugi::xpath_exception;

/** \brief Namespace containing the XPaths of the elements in the XML configuration file
 */
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

/** \brief Retrieve an int value attribute in the XML document
 *
 * \param xPath path: XPath of the XML element in which to read the value attribute
 * \return string: Retrieved int
 *
 */
int XmlParser::getIntValue(path xPath) const
{
	return atoi(getStringValue(xPath).c_str());
}