#ifndef XMLPARSER_H
#define XMLPARSER_H

#include <string>
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

	extern const fs::path ImageProcessing;
	extern const fs::path Camera;
	extern const fs::path Colors;
	extern const fs::path Color;
	extern const fs::path HSVcolor;
	extern const fs::path CircleSpec;

	extern const fs::path Motion;
	extern const fs::path Motors;
	extern const fs::path Head;
	extern const fs::path HorizontalHead;
	extern const fs::path VerticalHead;
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
	int getIntValue(fs::path xPath) const;
};

#endif // XMLPARSER_H
