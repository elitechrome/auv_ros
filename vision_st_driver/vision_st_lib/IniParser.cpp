#include "IniParser.h"

#include <cctype>
#include <cstdlib>



CIniParser::CIniParser(string filename)
{
	ini_parse(filename.c_str(), this);
}

CIniParser::~CIniParser(void)
{
}

/* Strip whitespace chars off end of given string, in place. Return s. */
char* CIniParser::rstrip(char* s)
{
    char* p = s + strlen(s);
    while (p > s && isspace((unsigned char)(*--p)))
        *p = '\0';
    return s;
}

/* Return pointer to first non-whitespace char in given string. */
char* CIniParser::lskip(const char* s)
{
    while (*s && isspace((unsigned char)(*s)))
        s++;
    return (char*)s;
}

/* Return pointer to first char c or ';' comment in given string, or pointer to
   null at end of string if neither found. ';' must be prefixed by a whitespace
   character to register as a comment. */
char* CIniParser::find_char_or_comment(const char* s, char c)
{
    int was_whitespace = 0;
    while (*s && *s != c && !(was_whitespace && *s == ';')) {
        was_whitespace = isspace((unsigned char)(*s));
        s++;
    }
    return (char*)s;
}

/* Version of strncpy that ensures dest (size bytes) is null-terminated. */
char* CIniParser::strncpy0(char* dest, const char* src, size_t size)
{
    strncpy(dest, src, size);
    dest[size - 1] = '\0';
    return dest;
}

/* See documentation in header file. */
int CIniParser::ini_parse_file(FILE* file, void* user)
{
    /* Uses a fair bit of stack (use heap instead if you need to) */

    char line[256];
    char section[64] = "";
    char prev_name[64] = "";

    char* start;
    char* end;
    char* name;
    char* value;
    int lineno = 0;
    int error = 0;

    /* Scan through file line by line */
    while (fgets(line, 256, file) != NULL) {
        lineno++;

        start = line;

#if INI_ALLOW_BOM
        if (lineno == 1 && (unsigned char)start[0] == 0xEF &&
                           (unsigned char)start[1] == 0xBB &&
                           (unsigned char)start[2] == 0xBF) {
            start += 3;
        }
#endif
        start = lskip(rstrip(start));

        if (*start == ';' || *start == '#') {
            /* Per Python ConfigParser, allow '#' comments at start of line */
        }

#if INI_ALLOW_MULTILINE
        else if (*prev_name && *start && start > line) {
            /* Non-black line with leading whitespace, treat as continuation
               of previous name's value (as per Python ConfigParser). */
            if (!handler(user, section, prev_name, start) && !error)
                error = lineno;
        }
#endif
        else if (*start == '[') {
            /* A "[section]" line */
            end = find_char_or_comment(start + 1, ']');
            if (*end == ']') {
                *end = '\0';
                strncpy0(section, start + 1, sizeof(section));
                *prev_name = '\0';
            }
            else if (!error) {
                /* No ']' found on section line */
                error = lineno;
            }
        }
        else if (*start && *start != ';') {
            /* Not a comment, must be a name[=:]value pair */
            end = find_char_or_comment(start, '=');
            if (*end != '=') {
                end = find_char_or_comment(start, ':');
            }
            if (*end == '=' || *end == ':') {
                *end = '\0';
                name = rstrip(start);
                value = lskip(end + 1);
                end = find_char_or_comment(value, '\0');
                if (*end == ';')
                    *end = '\0';
                rstrip(value);

                /* Valid name[=:]value pair found, call handler */
                strncpy0(prev_name, name, sizeof(prev_name));
                if (!ValueHandler(user, section, name, value) && !error)
                    error = lineno;
            }
            else if (!error) {
                /* No '=' or ':' found on name[=:]value line */
                error = lineno;
            }
        }

#if INI_STOP_ON_FIRST_ERROR
        if (error)
            break;
#endif
    }

    return error;
}

int CIniParser::ini_parse(const char* filename, void* user)
{
    FILE* file;
    int error;

    file = fopen(filename, "r");
    if (!file)
        return -1;
    error = ini_parse_file(file, user);
    fclose(file);
    return error;
}

string CIniParser::Get(string section, string name, string default_value)
{
    string key = MakeKey(section, name);
    return _values.count(key) ? _values[key] : default_value;
}

long CIniParser::GetInteger(string section, string name, long default_value)
{
    string valstr = Get(section, name, "");
    const char* value = valstr.c_str();
    char* end;
    // This parses "1234" (decimal) and also "0x4D2" (hex)
    long n = strtol(value, &end, 0);
    return end > value ? n : default_value;
}

double CIniParser::GetReal(string section, string name, double default_value)
{
    string valstr = Get(section, name, "");
    const char* value = valstr.c_str();
    char* end;
    double n = strtod(value, &end);
    return end > value ? n : default_value;
}

bool CIniParser::GetBoolean(string section, string name, bool default_value)
{
    string valstr = Get(section, name, "");
	char *str = (char *)valstr.c_str();

	int i=0;
	while(str[i])
	{
		str[i] = tolower(str[i]);
	}

    if (str == "true" || str == "yes" || str == "on" || str == "1")
        return true;
    else if (str == "false" || str == "no" || str == "off" || str == "0")
        return false;
    else
        return default_value;
}

string CIniParser::MakeKey(string section, string name)
{
    string key = section + "." + name;
	char *str = (char *)key.c_str();

	int i=0;
	while(str[i])
	{
		str[i] = tolower(str[i]);
		i++;
	}

    return key;
}

int CIniParser::ValueHandler(void* user, const char* section, const char* name, const char* value)
{
    CIniParser* reader = (CIniParser*)user;
    string key = MakeKey(section, name);
    if (reader->_values[key].size() > 0)
		reader->_values[key] += "\n";
    reader->_values[key] += value;
    return 1;
}

