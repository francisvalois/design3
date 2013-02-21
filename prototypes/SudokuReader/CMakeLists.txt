cmake_minimum_required (VERSION 2.6) 
project(SudokuReader)

file(GLOB_RECURSE Foo_SOURCES "*.cpp")
file(GLOB_RECURSE Foo_HEADERS "*.h")

set (Foo_INCLUDE_DIRS "")
foreach (_headerFile ${Foo_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND Foo_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES Foo_INCLUDE_DIRS)

find_package(OpenCV REQUIRED)
include_directories(${Foo_INCLUDE_DIRS})
add_executable(SudokuReader "SudokuReader.cpp" "NumberReader.cpp")
target_link_libraries(SudokuReader ${OpenCV_LIBS})
