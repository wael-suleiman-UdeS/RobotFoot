#ifndef XMLPARSER_H
#define XMLPARSER_H

#include <string>
#include "tinyxml2.h"

using std::string;
using namespace tinyxml2;

class ConfigParser
{
private:
	XMLDocument _doc;

public:
	ConfigParser();
	~ConfigParser();

	bool loadFile(string filePath);
	int getInt(string xPath);
};

#endif // XMLPARSER_H