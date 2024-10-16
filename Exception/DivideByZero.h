#pragma once
#include <stdexcept> 
#include <string>
using namespace std;

class DivideByZero:public invalid_argument{
    public:
    /**
     * @brief divide by zero constractor
     */
    explicit DivideByZero();
};