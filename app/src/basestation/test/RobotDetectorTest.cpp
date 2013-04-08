#include "gtest/gtest.h"
#include "opencv2/core/core.hpp"

#include "KinectUtility.h"
#include "RobotDetector.h"
#include "KinectTransformator.h"

using namespace cv;
using namespace std;

namespace { 
    class RobotDetectorTest : public ::testing::Test {
    protected:
        virtual void SetUp() {
            KinectTransformator::setKinectAngle(22.5);
            Vec2f kinectPosition(0.10f, -0.44f);
            KinectTransformator::setKinectPosition(kinectPosition);
        }

        RobotDetector kinect;
    };

    TEST_F(RobotDetectorTest, GetDistanceForRobotMatrix1){
        //Arrange
        Mat testMatrix = Utility::readFromFile("img/testKinect/robotdetection3.xml");
        Mat rgbMatrix = imread("img/testKinect/robotdetection3.jpg");
        float valueZ = 1.32f;
        float valueX = 0.6f; 
        float angleValue = -0.43f;
        
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

    TEST_F(RobotDetectorTest, GetDistanceForRobotMatrix2){
        //Arrange
        Mat testMatrix = Utility::readFromFile("img/testKinect/robotdetection4.xml");
        Mat rgbMatrix = imread("img/testKinect/robotdetection4.jpg");
        float valueZ = 1.91f;
        float valueX = 0.78f; 
        float angleValue = 0.39f;

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

    TEST_F(RobotDetectorTest, GetDistanceForRobotMatrix3){
        //Arrange
        Mat testMatrix = Utility::readFromFile("img/testKinect/robotdetection5.xml");
        Mat rgbMatrix = imread("img/testKinect/robotdetection5.jpg");
        float valueZ = 1.82f;
        float valueX = 0.38f; 
        float angleValue = -0.26;

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

    TEST_F(RobotDetectorTest, GetDistanceForRobotMatrix4){
        //Arrange
        Mat testMatrix = Utility::readFromFile("img/testKinect/robotdetection6.xml");
        Mat rgbMatrix = imread("img/testKinect/robotdetection6.jpg");
        float valueZ = 1.83f;
        float valueX = 0.37f; 
        float angleValue = -0.07;

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

