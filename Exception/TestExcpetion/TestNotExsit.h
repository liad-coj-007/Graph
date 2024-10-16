#pragma once
#include <stdexcept>
#include <string>
using namespace std;
class TestNotExsit:public invalid_argument{
    public:
    /**
     * @brief build test not exsit
     * @param testname - the test name that not exsit
     */
    explicit TestNotExsit(const string &testname);
};

/**
 * @brief get a testname and formatted the msg
 * @param testname - the name of the test
 * @return the formated msg
 */
string formatMessage(const std::string &testname);