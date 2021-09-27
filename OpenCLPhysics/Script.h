#pragma once

#include <string>

#define MAX_HITS_PER_OBJECT 100

#define TOSTRING(x) #x

class Script
{
    static std::string s_strOpenCLScript;
public:
    static const char* GetText();
private:
    static void Replace(std::string& str, const std::string& from, const std::string& to);
};
