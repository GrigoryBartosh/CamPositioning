#include <iostream>
#include <cstring>
#include "utils.h"
#include "my_algorythms.h"

using utils::checkInput;
using utils::unknownComand;

int main(int argc, char* argv[])
{
    if (checkInput(argc)) return 1;

    const char* type = argv[1];
    --argc;
    ++argv;
    
    if (strcmp(type, "rand") == 0)
    {
        return my_algorythms::main_rand();
    }
    else if (strcmp(type, "ransac") == 0) 
    {
        return my_algorythms::main_ransac();
    }
    else if (strcmp(type, "generate") == 0) 
    {
        return my_algorythms::main_generate(argc, argv);
    }
    else if (strcmp(type, "draw") == 0) 
    {
        return my_algorythms::main_draw();   
    }
    else 
    {
        return unknownComand();
    }
}
