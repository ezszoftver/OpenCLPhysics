#pragma once

#define TOSTRING(x) #x

class Script
{
    static const char* strOpenCLScript;
public:
    static const char* GetText();
};
