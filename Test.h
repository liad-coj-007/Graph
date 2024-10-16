using namespace std;
#include "Exception/TestExcpetion/FileError.h"
#include "Exception/TestExcpetion/InvaildName.h"
#include "Exception/TestExcpetion/TestNotExsit.h"
#include "Exception/DivideByZero.h"
#include <fstream>
#include <string>   // For std::string
#include <iostream>
#include <unordered_map>
#include <stdexcept> 
#include <sstream>



class Test{

    public:
    /**
     * @brief get the test singletone
     * @return a test object
     */
    static Test* GetTest();
    /**
     * @brief add test to data
     * @param output - the output the user get
     * @param expected_output_path - the expected output of the test path
     * @param name - the name of the test
     */
    void AddTest(const string &output,const string &expected_output_path,const string &testname = "");

    /**
     * @brief print test benchmark of name
     * @param name - the name of the test we print
     */
    void PrintTest(const string &testname)  ;

    /**
    * @brief write the banchmark to a test file 
    * @param testname - the name of the test we want to write
    */
    void WriteTest(const string &testname);

    /**
     * @brief return the precent of the test passed
     * @return the pass test rate
     */
    double GetPassTestRate()const ;


    //deleting cpy constaractor and assignment operator
    Test(const Test &othertest) = delete;
    Test& operator=(const Test &other) = delete;

    /**
     * @brief print all the tests
     * @param os - the os printer
     * @return test - the test we want to print
     */
    void PrintStats() const;

    template<class Function,class T>
    /**
     * @brief get a function and expected val and add test pass if there equal 
     * else it add to failure test
     * @param  function - the function that we turn
     * @param expectedval - the expected val we want to get
     * @param testname - the name of the test
     */
    void Assert(const Function &function,const T &expectedval,const string testname = ""){
        const string format_testname = getTestName(testname);
        T output = function();
        string actual = compareValues(expectedval,output);
        string expected = compareValues(expectedval,expectedval);
        AddBenchMark(actual,expected,format_testname);
    }

    template<class T>
     /**
     * @brief get a function and expected val and add test pass if there equal 
     * else it add to failure test
     * @param  function - the function that we turn
     * @param expectedval - the expected val we want to get
     * @param testname - the name of the test
     */
    void Assert(const T &output,const T &expectedval,const string testname = ""){
        const string format_testname = getTestName(testname);
        string actual = compareValues(expectedval,output);
        string expected = compareValues(expectedval,expectedval);
        AddBenchMark(actual,expected,format_testname);
    }
    
    private:
    unsigned int numoftestpass;
    unordered_map<string, string> data;
    /**
    * @brief get the test name of the test formated
    * @param name - the name of the test
    * @return the name of the test formated
    */
    string getTestName(const string name);

    /**
     * @brief Test constarctor
     */
    Test();
    /**
     * @brief add the benchmark to the data
     * @param output - the output we got
     * @param expected - the expected the string we want to get
     * @param testname - the name of the test
     */
    void AddBenchMark(const string &output,const string &expected,
    const string &testname);

    template <typename T>
    /**
     * @brief compare the values and return proper msg
     * @param expected - the expected val we want
     * @param output - the output we got
     */
    string compareValues(const T& expected, const T& output) {
        std::ostringstream oss;
        if (expected == output) {
            oss << output << " = " << expected;
        } else {
            oss << output << " != " << expected;
        }
        return oss.str();  // Convert the stream to a string
    }

};


