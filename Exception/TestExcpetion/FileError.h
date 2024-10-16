#pragma once
#include <stdexcept> 
#include <string>

using namespace std;
class FileError:public runtime_error{
    public:
    /**
     * @brief file error constractor
     * @param filename the name of the file we can't open
     */
    explicit FileError(const string &filename);

};