#include <iostream>

using std::cout;
using std::endl;
using std::runtime_error;

extern "C" float my_cpp_func(float a) 
{
    float f = a*a;
    cout << f << endl;
    return a*a;
}