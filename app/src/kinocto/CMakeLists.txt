cmake_minimum_required(VERSION 2.8.3)
project(kinocto)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)
find_package(OpenCV REQUIRED)
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

 add_message_files(
   FILES
 )

 add_service_files(
   FILES
   StartKinocto.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########
set(PROJ_SOURCES)

file(GLOB_RECURSE PROJ_SOURCES src/*.cpp)
file(GLOB_RECURSE PROJ_HEADERS include/${PROJ_NAME}/*.h)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
 #add_library(
 #KinoctoLib ${PROJ_SOURCES}
 #)

add_executable(Kinocto src/Kinocto.cpp)
add_dependencies(Kinocto kinocto_gencpp)
target_link_libraries(Kinocto ${catkin_LIBRARIES})

link_directories(include 
    ${catkin_LIB_DIRS}
)



#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS beginner_tutorials beginner_tutorials_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
#${PROJECT_NAME}
catkin_add_gtest(kinoctoTest test/AllTests.cpp 
src/Case.cpp src/Sudokube.cpp src/SudokubeSolver.cpp src/NumberReader.cpp 
)
target_link_libraries(kinoctoTest ${catkin_LIBRARIES} ${OpenCV_LIBS})
#catkin_add_executable() 
#catkin_add_gtest_build_flags()
#target_link_libraries(kinoctoTest ${catkin_LIBRARIES})

#if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
#endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#Tests pour l'extracteur de sudocube
#add_executable(SudocubeExtractor ${PROJ_SOURCES})
#target_link_libraries(SudocubeExtractor ${catkin_LIBRARIES} ${OpenCV_LIBS})

