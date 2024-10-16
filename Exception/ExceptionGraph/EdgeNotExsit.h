#pragma once
#include <stdexcept> 
#include <string>
using namespace std; 

class EdgeNotExsit :public runtime_error{
    public:
    EdgeNotExsit();
};