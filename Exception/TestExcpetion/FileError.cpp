#include "FileError.h"

const string msg = "Error: Unable to open file: ";
FileError::FileError(const string &filename):runtime_error(msg + filename){}