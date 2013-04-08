#include <gtest/gtest.h>

#include "KinectCalibratorTest.cpp"
#include "KinectTransformatorTest.cpp"
#include "ObjectDetectorTest.cpp"
#include "ObstacleDetectorTest.cpp"
#include "RobotDetectorTest.cpp"



int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
