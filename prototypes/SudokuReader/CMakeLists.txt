cmake_minimum_required (VERSION 2.6) 
project(SudokuReader)

find_package(OpenCV REQUIRED)
add_executable(SudokuReader  SudokuReader.cpp )
target_link_libraries(SudokuReader ${OpenCV_LIBS} )
