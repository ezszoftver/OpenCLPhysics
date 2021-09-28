#pragma once

#include <string>
#include <sstream>
#include <fstream>
#include <vector>

#define MAX_HITS_PER_OBJECT 100

class Script
{
public:
    static const char* GetText();
private:
    static void Replace(std::string& str, const std::string& from, const std::string& to);
    static bool LoadFileContents(std::string const& name, std::vector<char>& contents);

    static std::string s_strOpenCLScript;
};
