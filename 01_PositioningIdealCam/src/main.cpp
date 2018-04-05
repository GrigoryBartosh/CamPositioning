#include <iostream>
#include <cstring>
#include "my_algorythms.h"

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "Incorrect input format" << std::endl;
        return 1;
    }

    if (strcmp(argv[1], "rand") == 0)
    {
        my_algorythms::main_rand();
    }
    else if (strcmp(argv[1], "rand_short") == 0) 
    {
        my_algorythms::main_rand_short();
    }
    else if (strcmp(argv[1], "ransac") == 0) 
    {
        my_algorythms::main_ransac();
    }
    else if (strcmp(argv[1], "ransac_short") == 0) 
    {
        my_algorythms::main_ransac_short();   
    } 
    else if (strcmp(argv[1], "generate") == 0) 
    {
        my_algorythms::main_set_generator();
    } else 
    {
        std::cerr << "Unknown command" << std::endl;
        return 1;
    }

    return 0;
}
