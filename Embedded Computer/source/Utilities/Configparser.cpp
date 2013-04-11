#include "ConfigParser.h"
#include <unordered_map>

ConfigParser::ConfigParser()
{
}

ConfigParser::~ConfigParser()
{
}

bool ConfigParser::loadFile(string filePath)
{
	return !_doc.LoadFile(filePath.c_str());
}

int ConfigParser::getInt(string xPath)
{
	return 0;
}
