set(EXCEPTIONTEST_SOURCES FileError.cpp TestNotExsit.cpp InvaildName.cpp)  # Replace with actual filenames
# Create a library from these sources
add_library(ExceptionTestLib ${EXCEPTIONTEST_SOURCES})

target_include_directories(ExceptionTestLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})