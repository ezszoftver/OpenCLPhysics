#include "Script.h"

std::string Script::s_strOpenCLScript = "";

const char* Script::GetText() 
{
	std::ifstream f;
	f.open("OpenCLScript.txt");
	if (true == f.is_open())
	{
		std::stringstream strStream;
		strStream << f.rdbuf();
		s_strOpenCLScript = strStream.str();
		f.close();
	}
	else 
	{
		return nullptr;
	}

	Replace(s_strOpenCLScript, "#MAX_HITS#", std::to_string(MAX_HITS));
	Replace(s_strOpenCLScript, "#MAX_HITS_OBJECT_OBJECT#", std::to_string(MAX_HITS_OBJECT_OBJECT));

    return s_strOpenCLScript.c_str();
}

void Script::Replace(std::string& strRet, const std::string& strFrom, const std::string& strTo) 
{
	size_t nStart = 0;
	while ((nStart = strRet.find(strFrom, nStart)) != std::string::npos) 
	{
		strRet.replace(nStart, strFrom.length(), strTo);
		nStart += strTo.length();
	}
}
