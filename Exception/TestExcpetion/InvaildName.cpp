#include "InvaildName.h"
const string msg = "u already have a test that named: ";

InvaildName::InvaildName(const string &testname):invalid_argument(msg + testname){}