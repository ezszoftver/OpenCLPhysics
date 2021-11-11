#pragma once

#include <string>
#include <sstream>
#include <fstream>
#include <vector>

#define MAX_HITS 16
#define MAX_HITS_OBJECT_OBJECT 5

class Script
{
public:
    static const char* GetText();
private:
    static void Replace(std::string& str, const std::string& from, const std::string& to);
    static std::string s_strOpenCLScript;
};
