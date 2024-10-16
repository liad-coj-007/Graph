#include "TestNotExsit.h"

TestNotExsit::TestNotExsit(const std::string &testname)
    : std::invalid_argument(formatMessage(testname)) {}


std::string formatMessage(const std::string &testname) {
    return "The test " + testname + " doesn't exist on the system.";
}