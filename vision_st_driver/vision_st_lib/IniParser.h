#ifndef __INIPARSER_H
#define __INIPARSER_H

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <map>
#include <string>

using std::string;

class CIniParser
{
public:
    // Construct INIReader and parse given filename. 
	CIniParser(string filename);
	~CIniParser(void);

    // Get a string value from INI file, returning default_value if not found.
    std::string Get(string section, string name, string default_value);

    // Get an integer (long) value from INI file, returning default_value if
    // not found or not a valid integer (decimal "1234", "-1234", or hex "0x4d2").
    long GetInteger(string section, string name, long default_value);

    // Get a real (floating point double) value from INI file, returning
    // default_value if not found or not a valid floating point value
    // according to strtod().
    double GetReal(string section, string name, double default_value);

    // Get a boolean value from INI file, returning default_value if not found or if
    // not a valid true/false value. Valid true values are "true", "yes", "on", "1",
    // and valid false values are "false", "no", "off", "0" (not case sensitive).
    bool GetBoolean(string section, string name, bool default_value);

	char* rstrip(char* s);
	char* lskip(const char* s);
	char* find_char_or_comment(const char* s, char c);
	char* strncpy0(char* dest, const char* src, size_t size);
	int ini_parse_file(FILE* file, void* user);
	int ini_parse(const char* filename, void* user);


private:
	int _error;
    std::map<std::string, std::string> _values;
    string MakeKey(string section, string name);
    int ValueHandler(void* user, const char* section, const char* name, const char* value);

};

#endif
