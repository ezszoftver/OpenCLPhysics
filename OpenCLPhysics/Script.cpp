#include "Script.h"

const char* Script::strOpenCLScript = TOSTRING
(

__kernel void RefitTree(__global float* input, __global float* output, const unsigned int count)
{
    int id = get_global_id(0);
    if (id >= count)
    {
        return;
    }

    //output[id] = input[id] * input[id];
}

);

const char* Script::GetText() 
{
    return strOpenCLScript;
}
