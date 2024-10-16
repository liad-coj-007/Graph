#include "Test.h"
#include <iomanip> // Include this header for std::setprecision
#include <vector>

Test::Test(){
    numoftestpass = 0;
}

Test* Test::GetTest(){
    static Test test;
    return &test;
}


/**
 * @brief Return all the file data.
 * @param filename - The name of the file.
 * @return Contents of the file as a string.
 */
std::string readFileToString(const std::string& filename) {
    std::ifstream readfile(filename, std::ios::in | std::ios::ate); // Open file in read mode and seek to the end

    // Check if the file opened successfully
    if (!readfile) {
        throw FileError(filename);
    }

    std::streamsize size = readfile.tellg(); // Get the size of the file
    readfile.seekg(0); // Seek back to the beginning

    std::string content(size, ' '); // Create a string with the appropriate size
    readfile.read(&content[0], size); // Read the file content into the string

    return content; // Return the content
}

/**
 * @brief retturn a test name formatted
 * @param name - the name that need to formatted
 * @return formated string
 */
const string testformat = "Test ";
string Test::getTestName(const string name){
    string formatname;

    if(name == ""){
        int size = data.size()+1;    
        formatname =  testformat + to_string(size);
    }else{
        formatname = name;
    }

    if(data.find(formatname) != data.end()){
        throw InvaildName(formatname);
    }
    return formatname;  
}



/**
 * @brief compare two string and find the differnce between them
 * @param str1 - the first string we equal
 * @param str2 - the second string we equal
 * @return the diff between the strings
 */
string compareStrings(const std::string& str1, const std::string& str2) {
   std::stringstream ss1(str1);
    std::stringstream ss2(str2);
    std::string line1, line2;
    std::vector<std::string> lines1, lines2;

    // Read all lines into vectors
    while (std::getline(ss1, line1)) {
        lines1.push_back(line1);
    }
    while (std::getline(ss2, line2)) {
        lines2.push_back(line2);
    }

    std::string result;
    size_t maxLength = std::max(lines1.size(), lines2.size());
    for (size_t i = 0; i < maxLength; ++i) {
        if (i < lines1.size() && i < lines2.size()) {
            if (lines1[i] != lines2[i]) {
                result += "--- Line " + std::to_string(i + 1) + " ---\n";
                result += "- " + lines1[i] + "\n"; // Line from str1
                result += "+ " + lines2[i] + "\n"; // Line from str2
            }
        } else if (i < lines1.size()) {
            result += "--- Line " + std::to_string(i + 1) + " ---\n";
            result += "- " + lines1[i] + "\n"; // Line from str1 (removed)
        } else if (i < lines2.size()) {
            result += "--- Line " + std::to_string(i + 1) + " ---\n";
            result += "+ " + lines2[i] + "\n"; // Line from str2 (added)
        }
    }

    return result.empty() ? "No differences found." : result;
}



void  Test::AddTest(const string &output,const string &expected_output_path,const string &testname){
    const  string format_testname = getTestName(testname);
    string expected_output = readFileToString(expected_output_path);
    AddBenchMark(output,expected_output,format_testname);
}


void Test::AddBenchMark(const string &output,const string &expected,
    const string &testname){
    const string TESTPASS = "[TEST PASS!]";
    const string TESTFAILD = "[TEST Failed!]\n";
    const string EXPECTED = "Expected: ";
    const string ACTUAL = "Actual: ";
    const string DIFF = "Differences:\n";
    if (output == expected) {
        numoftestpass++;
        data[testname] = TESTPASS;
        return;
    } 
    string testfail_msg = TESTFAILD;
    testfail_msg += EXPECTED + expected + "\n";
    testfail_msg += ACTUAL + output + "\n";
    testfail_msg += DIFF;
    // Call the function to compare and display differences
    testfail_msg += compareStrings(expected, output);
    data[testname] = testfail_msg;
}

void Test::PrintTest(const string &testname) {
    if(data.find(testname) == data.end()){
        throw TestNotExsit(testname);
    }
    cout << testname << ": " << data[testname] << endl;
}

const string test_file_format = ".test_banchmark";
void Test::WriteTest(const string &testname) {
    if(!(data.find(testname) != data.end())){
        throw TestNotExsit(testname);
    }
    string path = testname + test_file_format;
    ofstream outFile(path);

    if (!outFile) {
        throw FileError(path);
    }
    // Write the string to the file
    outFile << data[testname];
    outFile.close();
}

double Test::GetPassTestRate()const {
    if(!data.size()){
        throw DivideByZero();
    }

    const int maxprecent = 100;
    return ((double)numoftestpass/data.size())*maxprecent;
}

const string space = "=================================================";
void Test::PrintStats()const {
    cout <<"we have " << numoftestpass << " succeed " << "out of " 
    << data.size() << endl;
    try{
        cout  << "The Pass Rate: " << fixed << std::setprecision(2) << 
        GetPassTestRate() <<"%"<<endl;
    }catch(const exception &e){
        cout  << "[DNE]" <<endl;    
    }
    for(const auto& pair: data){
        cout << space << endl;
        cout << pair.first << ": " << pair.second << endl;
    }
}