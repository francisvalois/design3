cmake_minimum_required (VERSION 2.6) 
project(SudokuReader)

find_package(OpenCV REQUIRED)
add_executable(CameraCapture  CameraCapture.cpp )
target_link_libraries(CameraCapture ${OpenCV_LIBS} )
