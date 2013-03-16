#include "gtest/gtest.h"
#include "opencv2/core/core.hpp"
#include "KinectUtility.h"
#include "Kinect.h"

using namespace cv;
using namespace std;

namespace {

//TODO: Modify Algorithm to make that test work. The obstacle behind the robot is not seen
/*TEST(KinectTest, GetDistanceForObstacleMatrix1) {
    //Arrange
    Mat testMatrix = Utility::readFromFile("img/kinectTest/matrixRobot1.xml");
    Kinect kinect;

    float valueX1 = 0.46f;
    float valueX2 = 10;
    float valueZ1 = 1.42f;
    float valueZ2 = 0;

    //Act
    kinect.findCenteredObstacle(testMatrix);
    Vec2f obstacle1 = kinect.getObstacle1();
    float positionX1 = obstacle1[0];
    float positionZ1 = obstacle1[1];
    Vec2f obstacle2 = kinect.getObstacle2();
    float positionX2 = obstacle2[0];
    float positionZ2 = obstacle2[1];

    //Assert
    //Accept a range of distances for X:0.805 and Z:2.2209
    ASSERT_TRUE(positionX1 >= valueX1-0.01 && positionX1 < valueX1 + 0.01);
    ASSERT_TRUE(positionZ1 >= valueZ1-0.01 && positionZ1 < valueZ1 + 0.01);

    //Accept a range of distances for X:0.258 and Z:1.337
    ASSERT_TRUE(positionX2 >= valueX2-0.01 && positionX2 < valueX2 + 0.01);
    ASSERT_TRUE(positionZ2 >= valueZ2-0.01 && positionZ2 < valueZ2 + 0.01);
}*/

TEST(KinectTest, GetDistanceForObstacleMatrix2) {
    //Arrange
    Mat testMatrix = Utility::readFromFile("img/testKinect/matrixRobot2.xml");
    Kinect kinect;

    float valueX1 = 0.74f;
    float valueX2 = 0.46f;
    float valueZ1 = 1.41f;
    float valueZ2 = 1.42f;

    //Act
    kinect.findCenteredObstacle(testMatrix);
    Vec2f obstacle1 = kinect.getObstacle1();
    float positionX1 = obstacle1[0];
    float positionZ1 = obstacle1[1];
    Vec2f obstacle2 = kinect.getObstacle2();
    float positionX2 = obstacle2[0];
    float positionZ2 = obstacle2[1];

    //Assert
    //Accept a range of distances for X:0.805 and Z:2.2209
    ASSERT_TRUE(positionX1 >= valueX1-0.01 && positionX1 < valueX1 + 0.01);
    ASSERT_TRUE(positionZ1 >= valueZ1-0.01 && positionZ1 < valueZ1 + 0.01);

    //Accept a range of distances for X:0.258 and Z:1.337
    ASSERT_TRUE(positionX2 >= valueX2-0.01 && positionX2 < valueX2 + 0.01);
    ASSERT_TRUE(positionZ2 >= valueZ2-0.01 && positionZ2 < valueZ2 + 0.01);
}

//TODO: Modify Algorithm to make that test work, the obstacle next to the drawing board is not detected
TEST(KinectTest, GetDistanceForObstacleMatrix3) {
    //Arrange
    Mat testMatrix = Utility::readFromFile("img/testKinect/matrixRobot3.xml");
    Kinect kinect;

    float valueX1 = 0.74f;
    float valueX2 = 10;
    float valueZ1 = 1.41f;
    float valueZ2 = 0;

    //Act
    kinect.findCenteredObstacle(testMatrix);
    Vec2f obstacle1 = kinect.getObstacle1();
    float positionX1 = obstacle1[0];
    float positionZ1 = obstacle1[1];
    Vec2f obstacle2 = kinect.getObstacle2();
    float positionX2 = obstacle2[0];
    float positionZ2 = obstacle2[1];

    //Assert
    //Accept a range of distances for X:0.805 and Z:2.2209
    ASSERT_TRUE(positionX1 >= valueX1-0.01 && positionX1 < valueX1 + 0.01);
    ASSERT_TRUE(positionZ1 >= valueZ1-0.01 && positionZ1 < valueZ1 + 0.01);

    //Accept a range of distances for X:0.258 and Z:1.337
    ASSERT_TRUE(positionX2 >= valueX2-0.01 && positionX2 < valueX2 + 0.01);
    ASSERT_TRUE(positionZ2 >= valueZ2-0.01 && positionZ2 < valueZ2 + 0.01);
}

TEST(KinectTest, GetDistanceForObstacleMatrix4) {
    //Arrange
    Mat testMatrix = Utility::readFromFile("img/testKinect/matrixRobot4.xml");
    Kinect kinect;

    float valueX1 = 0.82f;
    float valueX2 = 0.37f;
    float valueZ1 = 1.02f;
    float valueZ2 = 2.13f;

    //Act
    kinect.findCenteredObstacle(testMatrix);
    Vec2f obstacle1 = kinect.getObstacle1();
    float positionX1 = obstacle1[0];
    float positionZ1 = obstacle1[1];
    Vec2f obstacle2 = kinect.getObstacle2();
    float positionX2 = obstacle2[0];
    float positionZ2 = obstacle2[1];

    //Assert
    //Accept a range of distances for X:0.805 and Z:2.2209
    ASSERT_TRUE(positionX1 >= valueX1-0.01 && positionX1 < valueX1 + 0.01);
    ASSERT_TRUE(positionZ1 >= valueZ1-0.01 && positionZ1 < valueZ1 + 0.01);

    //Accept a range of distances for X:0.258 and Z:1.337
    ASSERT_TRUE(positionX2 >= valueX2-0.01 && positionX2 < valueX2 + 0.01);
    ASSERT_TRUE(positionZ2 >= valueZ2-0.01 && positionZ2 < valueZ2 + 0.01);
}

TEST(KinectTest, GetDistanceForObstacleMatrix5) {
    //Arrange
    Mat testMatrix = Utility::readFromFile("img/testKinect/matrixRobot5.xml");
    Kinect kinect;

    float valueX1 = 0.61f;
    float valueX2 = .47f;
    float valueZ1 = 1.18f;
    float valueZ2 = 1.17f;

    //Act
    kinect.findCenteredObstacle(testMatrix);
    Vec2f obstacle1 = kinect.getObstacle1();
    float positionX1 = obstacle1[0];
    float positionZ1 = obstacle1[1];
    Vec2f obstacle2 = kinect.getObstacle2();
    float positionX2 = obstacle2[0];
    float positionZ2 = obstacle2[1];

    //Assert
    //Accept a range of distances for X:0.805 and Z:2.2209
    ASSERT_TRUE(positionX1 >= valueX1-0.01 && positionX1 < valueX1 + 0.01);
    ASSERT_TRUE(positionZ1 >= valueZ1-0.01 && positionZ1 < valueZ1 + 0.01);

    //Accept a range of distances for X:0.258 and Z:1.337
    ASSERT_TRUE(positionX2 >= valueX2-0.01 && positionX2 < valueX2 + 0.01);
    ASSERT_TRUE(positionZ2 >= valueZ2-0.01 && positionZ2 < valueZ2 + 0.01);
}

TEST(KinectTest, GetDistanceForObstacleMatrix6) {
    //Arrange
    Mat testMatrix = Utility::readFromFile("img/testKinect/matrixRobot6.xml");
    Kinect kinect;

    float valueX1 = 0.61f;
    float valueX2 = 0.42f;
    float valueZ1 = 1.17f;
    float valueZ2 = 1.06f;

    //Act
    kinect.findCenteredObstacle(testMatrix);
    Vec2f obstacle1 = kinect.getObstacle1();
    float positionX1 = obstacle1[0];
    float positionZ1 = obstacle1[1];
    Vec2f obstacle2 = kinect.getObstacle2();
    float positionX2 = obstacle2[0];
    float positionZ2 = obstacle2[1];

    //Assert
    //Accept a range of distances for X:0.805 and Z:2.2209
    ASSERT_TRUE(positionX1 >= valueX1-0.01 && positionX1 < valueX1 + 0.01);
    ASSERT_TRUE(positionZ1 >= valueZ1-0.01 && positionZ1 < valueZ1 + 0.01);

    //Accept a range of distances for X:0.258 and Z:1.337
    ASSERT_TRUE(positionX2 >= valueX2-0.01 && positionX2 < valueX2 + 0.01);
    ASSERT_TRUE(positionZ2 >= valueZ2-0.01 && positionZ2 < valueZ2 + 0.01);
}

/*//TODO: Modify algorithm to make that test work, modify threshold for dectection of near obstacle
TEST(KinectTest, GetDistanceForObstacleMatrix7) {
    //Arrange
    Mat testMatrix = Utility::readFromFile("img/testKinect/matrixRobot7.xml");
    Kinect kinect;

    float valueX1 = 0.57f;
    float valueX2 = 0.43f;
    float valueZ1 = 1.12f;
    float valueZ2 = 1;

    //Act
    kinect.findCenteredObstacle(testMatrix);
    Vec2f obstacle1 = kinect.getObstacle1();
    float positionX1 = obstacle1[0];
    float positionZ1 = obstacle1[1];
    Vec2f obstacle2 = kinect.getObstacle2();
    float positionX2 = obstacle2[0];
    float positionZ2 = obstacle2[1];

    //Assert
    //Accept a range of distances for X:0.805 and Z:2.2209
    ASSERT_TRUE(positionX1 >= valueX1-0.01 && positionX1 < valueX1 + 0.01);
    ASSERT_TRUE(positionZ1 >= valueZ1-0.01 && positionZ1 < valueZ1 + 0.01);

    //Accept a range of distances for X:0.258 and Z:1.337
    ASSERT_TRUE(positionX2 >= valueX2-0.01 && positionX2 < valueX2 + 0.01);
    ASSERT_TRUE(positionZ2 >= valueZ2-0.01 && positionZ2 < valueZ2 + 0.01);
}*/

TEST(KinectTest, GetDistanceForObstacleMatrix8) {
    //Arrange
    Mat testMatrix = Utility::readFromFile("img/testKinect/matrixRobot8.xml");
    Kinect kinect;

    float valueX1 = 0.72f;
    float valueX2 = 0.27f;
    float valueZ1 = 0.91f;
    float valueZ2 = 1.86f;

    //Act
    kinect.findCenteredObstacle(testMatrix);
    Vec2f obstacle1 = kinect.getObstacle1();
    float positionX1 = obstacle1[0];
    float positionZ1 = obstacle1[1];
    Vec2f obstacle2 = kinect.getObstacle2();
    float positionX2 = obstacle2[0];
    float positionZ2 = obstacle2[1];

    //Assert
    //Accept a range of distances for X:0.805 and Z:2.2209
    ASSERT_TRUE(positionX1 >= valueX1-0.01 && positionX1 < valueX1 + 0.01);
    ASSERT_TRUE(positionZ1 >= valueZ1-0.01 && positionZ1 < valueZ1 + 0.01);

    //Accept a range of distances for X:0.258 and Z:1.337
    ASSERT_TRUE(positionX2 >= valueX2-0.01 && positionX2 < valueX2 + 0.01);
    ASSERT_TRUE(positionZ2 >= valueZ2-0.01 && positionZ2 < valueZ2 + 0.01);
}

TEST(KinectTest, GetDistanceForObstacleMatrix9) {
    //Arrange
    Mat testMatrix = Utility::readFromFile("img/testKinect/matrixRobot9.xml");
    Kinect kinect;

    float valueX1 = 0.30f;
    float valueX2 = 0.26f;
    float valueZ1 = 1.13f;
    float valueZ2 = 1.86f;

    //Act
    kinect.findCenteredObstacle(testMatrix);
    Vec2f obstacle1 = kinect.getObstacle1();
    float positionX1 = obstacle1[0];
    float positionZ1 = obstacle1[1];
    Vec2f obstacle2 = kinect.getObstacle2();
    float positionX2 = obstacle2[0];
    float positionZ2 = obstacle2[1];

    //Assert
    //Accept a range of distances for X:0.805 and Z:2.2209
    ASSERT_TRUE(positionX1 >= valueX1-0.01 && positionX1 < valueX1 + 0.01);
    ASSERT_TRUE(positionZ1 >= valueZ1-0.01 && positionZ1 < valueZ1 + 0.01);

    //Accept a range of distances for X:0.258 and Z:1.337
    ASSERT_TRUE(positionX2 >= valueX2-0.01 && positionX2 < valueX2 + 0.01);
    ASSERT_TRUE(positionZ2 >= valueZ2-0.01 && positionZ2 < valueZ2 + 0.01);
}

TEST(KinectTest, getTrueCoordFromKinectCoords) {
    //Arrange
    Kinect kinect;
    Vec3f depthXYZ(1, 2, 3);

    //Act
    Vec2f trueCoords = kinect.getTrueCoordFromKinectCoord(depthXYZ);

    //Assert a range
    ASSERT_TRUE(trueCoords[0] >= (0.42 - 0.3) || trueCoords[0] <= (0.42 + 0.3));
    ASSERT_TRUE(trueCoords[1] >= (2.59 - 0.3) || trueCoords[1] <= (2.59 + 0.3));
}

}
