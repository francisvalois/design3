#include "gtest/gtest.h"
#include "opencv2/core/core.hpp"

#include "KinectUtility.h"
#include "RobotDetector.h"

using namespace cv;
using namespace std;

namespace {
    class RobotDetectorTest : public ::testing::Test {
    protected:
        Mat testMatrix;
        Mat rgbMatrix;
        RobotDetector kinect;
    };

    TEST_F(RobotDetectorTest, GetDistanceForRobotMatrix1Table2){
        //Arrange
        testMatrix = Utility::readFromFile("table2/matrice1.xml");
        rgbMatrix = imread("table2/rgb1.jpg");
        float valueZ = 0.28f;
        float valueX = 0.97f;
        float angleValue = 0;

        //Act
        kinect.findRobotWithAngle(testMatrix, rgbMatrix);
        Vec2f obstacle1 = kinect.getRobotPosition();
        float positionX = obstacle1[0];
        float positionZ = obstacle1[1];
        float robotAngle = kinect.getRobotAngle();

        //Assert
        ASSERT_TRUE(positionX >= valueX - 0.03 && positionX <= valueX + 0.03);
        ASSERT_TRUE(positionZ >= valueZ - 0.03 && positionZ <= valueZ + 0.03);

        ASSERT_TRUE(robotAngle >= angleValue - 0.05 && robotAngle <= angleValue + 0.05);
    }

    TEST_F(RobotDetectorTest, GetDistanceForRobotMatrix2Table2){
        //Arrange
         testMatrix = Utility::readFromFile("table2/matrice3.xml");
         rgbMatrix = imread("table2/rgb3.jpg");
        float valueZ = 0.55f;
        float valueX = 0.555f;
        float angleValue = 1.57f;

        //Act
        kinect.findRobotWithAngle(testMatrix, rgbMatrix);
        Vec2f obstacle1 = kinect.getRobotPosition();
        float positionX = obstacle1[0];
        float positionZ = obstacle1[1];
        float robotAngle = kinect.getRobotAngle();

        //Assert
        ASSERT_TRUE(positionX >= valueX - 0.03 && positionX <= valueX + 0.03);
        ASSERT_TRUE(positionZ >= valueZ - 0.03 && positionZ <= valueZ + 0.03);

        ASSERT_TRUE(robotAngle >= angleValue - 0.05 && robotAngle <= angleValue + 0.05);
    }

    TEST_F(RobotDetectorTest, GetDistanceForRobotMatrix1Table1){
        //Arrange
        testMatrix = Utility::readFromFile("table1/matrice1.xml");
        rgbMatrix = imread("table1/rgb1.jpg");
        float valueZ = 0.20f;
        float valueX = 0.97f;
        float angleValue = 0;

        //Act
        kinect.findRobotWithAngle(testMatrix, rgbMatrix);
        Vec2f obstacle1 = kinect.getRobotPosition();
        float positionX = obstacle1[0];
        float positionZ = obstacle1[1];
        float robotAngle = kinect.getRobotAngle();


        //Assert
        ASSERT_TRUE(positionX >= valueX - 0.03 && positionX <= valueX + 0.03);
        ASSERT_TRUE(positionZ >= valueZ - 0.03 && positionZ <= valueZ + 0.03);

        ASSERT_TRUE(robotAngle >= angleValue - 0.05 && robotAngle <= angleValue + 0.05);
    }

    TEST_F(RobotDetectorTest, GetDistanceForRobotMatrix2Table1){
        //Arrange
        testMatrix = Utility::readFromFile("table1/matrice4.xml");
        rgbMatrix = imread("table1/rgb4.jpg");
        float valueZ = 0.53f;
        float valueX = 0.57f;
        float angleValue = 1.55f;

        //Act
        kinect.findRobotWithAngle(testMatrix, rgbMatrix);
        Vec2f obstacle1 = kinect.getRobotPosition();
        float positionX = obstacle1[0];
        float positionZ = obstacle1[1];
        float robotAngle = kinect.getRobotAngle();


        //Assert
        ASSERT_TRUE(positionX >= valueX - 0.03 && positionX <= valueX + 0.03);
        ASSERT_TRUE(positionZ >= valueZ - 0.03 && positionZ <= valueZ + 0.03);

        ASSERT_TRUE(robotAngle >= angleValue - 0.05 && robotAngle <= angleValue + 0.05);
    }
}

