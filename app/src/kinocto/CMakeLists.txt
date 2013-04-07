cmake_minimum_required(VERSION 2.8.3)
project(kinocto)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)
find_package(OpenCV REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

 add_message_files(
   FILES
 )

add_service_files(
  FILES
  TestExtractSudocubeAndSolve.srv
  TestFindRobotAngle.srv
  TestFindRobotPosition.srv
  TestGetAntennaParamAndShow.srv
  TestGoToSudocubeX.srv
  TestFindObstacles.srv
  TestDrawNumber.srv
  TestGoToGreenFrameAndDraw.srv
  TestAdjustFrontPosition.srv
  TestAdjustSidePosition.srv
  TestAdjustAngle.srv
  TestAdjustSidePositionWithGreenFrame.srv
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
#file(GLOB_RECURSE PROJ_HEADERS include/${PROJ_NAME}/*.h)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(Kinocto Kinocto.cpp ${PROJ_SOURCES})
add_dependencies(Kinocto kinocto_gencpp basestation_gencpp microcontroller_gencpp)
target_link_libraries(Kinocto ${catkin_LIBRARIES} ${OpenCV_LIBS})

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

catkin_add_gtest(kinoctoTest test/AllTests.cpp 
    "src/sudocube/Case.cpp"
    "src/sudocube/Sudocube.cpp"
    "src/sudocube/SudocubeSolver.cpp"
    "src/pathPlanning/PathPlanning.cpp"
    "src/pathPlanning/Node.cpp"
    "src/vision/NumberReader.cpp"
    "src/vision/WallAngleFinder.cpp"
    "src/vision/FrameCenterFinder.cpp"
    "src/Workspace.cpp"
)
target_link_libraries(kinoctoTest ${OpenCV_LIBS} ${catkin_LIBRARIES})


catkin_add_gtest(kinoctoLongTest test/AllLongTests.cpp 
    "src/vision/NumberReader.cpp"
    "src/vision/SudocubeExtractor.cpp"
    "src/vision/VisionUtility.cpp"
    "src/vision/GreenFrameExtractor.cpp"
    "src/vision/SquaresExtractor.cpp"
    "src/sudocube/Sudocube.cpp"
    "src/sudocube/Case.cpp" 
)
target_link_libraries(kinoctoLongTest ${OpenCV_LIBS})
