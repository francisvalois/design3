#include <gtest/gtest.h>

#include "sudocube/CaseTest.cpp"
#include "sudocube/SudocubeTest.cpp"
#include "sudocube/SudocubeSolverTest.cpp"
#include "pathPlanning/PathPlanningTest.cpp"
#include "vision/NumberReaderIT.cpp"
#include "vision/WallAngleFinderIT.cpp"
#include "vision/FrameCenterFinderIT.cpp"
#include "vision/GreenBorderAngleFinderIT.cpp"

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

