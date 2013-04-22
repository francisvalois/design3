#include "gtest/gtest.h"
#include "opencv2/core/core.hpp"

#include "KinectUtility.h"
#include "ObstaclesDetector.h"

using namespace cv;
using namespace std;

namespace {
    class ObstaclesDetectorTest : public ::testing::Test {
    protected:
        Mat testMatrix;
        Mat testRGB;
        ObstaclesDetector kinect;
    };

    TEST_F(ObstaclesDetectorTest, GetDistanceForObstacleMatrix1Table2){
        //Arrange
        testMatrix = Utility::readFromFile("table2/matrice2.xml");

        float valueZ2 = 2.30f;
        float valueZ1 = 2.30f;
        float valueX2 = 0.05f;
        float valueX1 = 0.97f;
        
        //Act
        kinect.findCenteredObstacle(testMatrix);
        Vec2f obstacle1 = kinect.getObstacle1();
        float positionX1 = obstacle1[0];
        float positionZ1 = obstacle1[1];
        Vec2f obstacle2 = kinect.getObstacle2();
        float positionX2 = obstacle2[0];
        float positionZ2 = obstacle2[1];
        
        //Assert
        ASSERT_TRUE(positionX1 >= valueX1 - 0.05 && positionX1 <= valueX1 + 0.05);
        ASSERT_TRUE(positionZ1 >= valueZ1 - 0.05 && positionZ1 <= valueZ1 + 0.05);
    
        ASSERT_TRUE(positionX2 >= valueX2 - 0.05 && positionX2 <= valueX2 + 0.05);
        ASSERT_TRUE(positionZ2 >= valueZ2 - 0.05 && positionZ2 <= valueZ2 + 0.05);
    }

    TEST_F(ObstaclesDetectorTest, GetDistanceForObstacleMatrix2Table2){
    //Arrange
    testMatrix = Utility::readFromFile("table2/matrice4.xml");

    float valueZ2 = 1.64f;
    float valueZ1 = 1.61f;
    float valueX2 = 0.07f;
    float valueX1 = 1.00f;

    //Act
    kinect.findCenteredObstacle(testMatrix);
    Vec2f obstacle1 = kinect.getObstacle1();
    float positionX1 = obstacle1[0];
    float positionZ1 = obstacle1[1];
    Vec2f obstacle2 = kinect.getObstacle2();
    float positionX2 = obstacle2[0];
    float positionZ2 = obstacle2[1];

    //Assert
    ASSERT_TRUE(positionX1 >= valueX1 - 0.04 && positionX1 <= valueX1 + 0.04);
    ASSERT_TRUE(positionZ1 >= valueZ1 - 0.04 && positionZ1 <= valueZ1 + 0.04);

    ASSERT_TRUE(positionX2 >= valueX2 - 0.04 && positionX2 <= valueX2 + 0.04);
    ASSERT_TRUE(positionZ2 >= valueZ2 - 0.04 && positionZ2 <= valueZ2 + 0.04);
}

    TEST_F(ObstaclesDetectorTest, GetDistanceForObstacleMatrix3Table2){
    //Arrange
    testMatrix = Utility::readFromFile("table2/matrice5.xml");

    float valueZ2 = 1.285f;
    float valueZ1 = 1.287f;
    float valueX2 = 0.224f;
    float valueX1 = 0.83f;

    //Act
    kinect.findCenteredObstacle(testMatrix);
    Vec2f obstacle1 = kinect.getObstacle1();
    float positionX1 = obstacle1[0];
    float positionZ1 = obstacle1[1];
    Vec2f obstacle2 = kinect.getObstacle2();
    float positionX2 = obstacle2[0];
    float positionZ2 = obstacle2[1];

    //Assert
    ASSERT_TRUE(positionX1 >= valueX1 - 0.03 && positionX1 <= valueX1 + 0.03);
    ASSERT_TRUE(positionZ1 >= valueZ1 - 0.03 && positionZ1 <= valueZ1 + 0.03);

    ASSERT_TRUE(positionX2 >= valueX2 - 0.03 && positionX2 <= valueX2 + 0.03);
    ASSERT_TRUE(positionZ2 >= valueZ2 - 0.03 && positionZ2 <= valueZ2 + 0.03);
}

    TEST_F(ObstaclesDetectorTest, GetDistanceForObstacleMatrix2Table1){
    //Arrange
    testMatrix = Utility::readFromFile("table1/matrice3.xml");

    float valueZ2 = 1.66f;
    float valueZ1 = 1.57f;
    float valueX2 = 0.07f;
    float valueX1 = 0.98f;

    //Act
    kinect.findCenteredObstacle(testMatrix);
    Vec2f obstacle1 = kinect.getObstacle1();
    float positionX1 = obstacle1[0];
    float positionZ1 = obstacle1[1];
    Vec2f obstacle2 = kinect.getObstacle2();
    float positionX2 = obstacle2[0];
    float positionZ2 = obstacle2[1];

    //Assert
    ASSERT_TRUE(positionX1 >= valueX1 - 0.04 && positionX1 <= valueX1 + 0.04);
    ASSERT_TRUE(positionZ1 >= valueZ1 - 0.04 && positionZ1 <= valueZ1 + 0.04);

    ASSERT_TRUE(positionX2 >= valueX2 - 0.04 && positionX2 <= valueX2 + 0.04);
    ASSERT_TRUE(positionZ2 >= valueZ2 - 0.04 && positionZ2 <= valueZ2 + 0.04);
}

    TEST_F(ObstaclesDetectorTest, GetDistanceForObstacleMatrix3Table1){
    //Arrange
    testMatrix = Utility::readFromFile("table1/matrice5.xml");

    float valueZ2 = 1.35f;
    float valueZ1 = 1.29f;
    float valueX2 = 0.21f;
    float valueX1 = 0.83f;

    //Act
    kinect.findCenteredObstacle(testMatrix);
    Vec2f obstacle1 = kinect.getObstacle1();
    float positionX1 = obstacle1[0];
    float positionZ1 = obstacle1[1];
    Vec2f obstacle2 = kinect.getObstacle2();
    float positionX2 = obstacle2[0];
    float positionZ2 = obstacle2[1];

    //Assert
    ASSERT_TRUE(positionX1 >= valueX1 - 0.03 && positionX1 <= valueX1 + 0.03);
    ASSERT_TRUE(positionZ1 >= valueZ1 - 0.03 && positionZ1 <= valueZ1 + 0.03);

    ASSERT_TRUE(positionX2 >= valueX2 - 0.03 && positionX2 <= valueX2 + 0.03);
    ASSERT_TRUE(positionZ2 >= valueZ2 - 0.03 && positionZ2 <= valueZ2 + 0.03);
}


}

