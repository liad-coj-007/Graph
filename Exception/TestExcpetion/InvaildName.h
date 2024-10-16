#pragma once
#include <stdexcept> 
#include <string>
using namespace std;

class InvaildName:public invalid_argument{
    public:
    /**
     * @brief constractor of invaildname object
     * @param testname - the testname the program found invaild
     */
    explicit InvaildName(const string &testname);
};