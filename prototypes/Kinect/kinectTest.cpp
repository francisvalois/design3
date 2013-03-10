#include "gtest/gtest.h"
#include "opencv2/core/core.hpp"
#include "Utility.h"
#include "kinect.h"

using namespace cv;
using namespace std;

namespace {
    
    TEST(KinectTest, GetDistanceForRobotMatrix1){
    //Arrange

    // Mock obstacle
    Vec2f obstacle1(0.46f, 1.41f);
    Vec2f obstacle2;

    Mat testMatrix = Utility::readFromFile("matrixRobot1.xml");
    Kinect kinect(obstacle1, obstacle2);

    float valueX = 0.484f;
    float valueZ = 0.47f;

    //Act
    kinect.findRobot(testMatrix);
    Vec2f robot = kinect.getRobot();
    float positionX = robot[0];
    float positionZ = robot[1];


    //Assert
    ASSERT_TRUE(positionX >= valueX-0.01 && positionX < valueX + 0.01);
    ASSERT_TRUE(positionZ >= valueZ-0.01 && positionZ < valueZ + 0.01);
    }
    
    TEST(KinectTest, GetDistanceForRobotMatrix2){
        //Arrange
        
        // Mock obstacle
        Vec2f obstacle1(0.74f, 1.415f);
        Vec2f obstacle2(0.463f, 1.42f);
        
        Mat testMatrix = Utility::readFromFile("matrixRobot2.xml");
        Kinect kinect(obstacle1, obstacle2);
        
        float valueX = 0.289f;
        float valueZ = 1.027f;
        
        //Act
        kinect.findRobot(testMatrix);
        Vec2f robot = kinect.getRobot();
        float positionX = robot[0];
        float positionZ = robot[1];
        
        
        //Assert
        ASSERT_TRUE(positionX >= valueX-0.01 && positionX < valueX + 0.01);
        ASSERT_TRUE(positionZ >= valueZ-0.01 && positionZ < valueZ + 0.01);
    }
    
    TEST(KinectTest, GetDistanceForRobotMatrix3){
        //Arrange
        
        // Mock obstacle
        Vec2f obstacle1(0.74f, 1.404f);
        Vec2f obstacle2;
        
        Mat testMatrix = Utility::readFromFile("matrixRobot3.xml");
        Kinect kinect(obstacle1, obstacle2);
        
        float valueX = 0.324f;
        float valueZ = 1.002f;
        
        //Act
        kinect.findRobot(testMatrix);
        Vec2f robot = kinect.getRobot();
        float positionX = robot[0];
        float positionZ = robot[1];
        
        
        //Assert
        ASSERT_TRUE(positionX >= valueX-0.01 && positionX < valueX + 0.01);
        ASSERT_TRUE(positionZ >= valueZ-0.01 && positionZ < valueZ + 0.01);
    }

    //TODO : Correct the robotPosition. The black border makes some error in median calculation
    TEST(KinectTest, GetDistanceForRobotMatrix4){
        //Arrange
        
        // Mock obstacle
        Vec2f obstacle1(0.815f, 1.008f);
        Vec2f obstacle2(0.367f, 2.107f);
        
        Mat testMatrix = Utility::readFromFile("matrixRobot4.xml");
        Kinect kinect(obstacle1, obstacle2);
        
        float valueX = 0.0f;
        float valueZ = 0.0f;
        
        //Act
        kinect.findRobot(testMatrix);
        Vec2f robot = kinect.getRobot();
        float positionX = robot[0];
        float positionZ = robot[1];
        
        
        //Assert
        ASSERT_TRUE(positionX >= valueX-0.01 && positionX < valueX + 0.01);
        ASSERT_TRUE(positionZ >= valueZ-0.01 && positionZ < valueZ + 0.01);
    }
    
    TEST(KinectTest, GetDistanceForRobotMatrix5){
        //Arrange
        
        // Mock obstacle
        Vec2f obstacle1(0.602f, 1.164f);
        Vec2f obstacle2(0.468f, 1.167f);
        
        Mat testMatrix = Utility::readFromFile("matrixRobot5.xml");
        Kinect kinect(obstacle1, obstacle2);
        
        float valueX = 0.0f;
        float valueZ = 0.0f;
        
        //Act
        kinect.findRobot(testMatrix);
        Vec2f robot = kinect.getRobot();
        float positionX = robot[0];
        float positionZ = robot[1];
        
        
        //Assert
        ASSERT_TRUE(positionX >= valueX-0.01 && positionX < valueX + 0.01);
        ASSERT_TRUE(positionZ >= valueZ-0.01 && positionZ < valueZ + 0.01);
    }
    
    TEST(KinectTest, GetDistanceForRobotMatrix6){
        //Arrange
        
        // Mock obstacle       
        Vec2f obstacle1(0.607f, 1.161f);
        Vec2f obstacle2(0.421f, 1.046f);
        
        Mat testMatrix = Utility::readFromFile("matrixRobot6.xml");
        Kinect kinect(obstacle1, obstacle2);
        
        float valueX = 0.0f;
        float valueZ = 0.0f;
        
        //Act
        kinect.findRobot(testMatrix);
        Vec2f robot = kinect.getRobot();
        float positionX = robot[0];
        float positionZ = robot[1];
        
        
        //Assert
        ASSERT_TRUE(positionX >= valueX-0.01 && positionX < valueX + 0.01);
        ASSERT_TRUE(positionZ >= valueZ-0.01 && positionZ < valueZ + 0.01);
    }
    
    TEST(KinectTest, GetDistanceForRobotMatrix7){
        //Arrange
        
        // Mock obstacle
        Vec2f obstacle1(0.583f, 1.164f);
        Vec2f obstacle2(0.422f, 1.04f);
        
        Mat testMatrix = Utility::readFromFile("matrixRobot7.xml");
        Kinect kinect(obstacle1, obstacle2);
        
        float valueX = 0.0f;
        float valueZ = 0.0f;
        
        //Act
        kinect.findRobot(testMatrix);
        Vec2f robot = kinect.getRobot();
        float positionX = robot[0];
        float positionZ = robot[1];
        
        
        //Assert
        ASSERT_TRUE(positionX >= valueX-0.01 && positionX < valueX + 0.01);
        ASSERT_TRUE(positionZ >= valueZ-0.01 && positionZ < valueZ + 0.01);
    }
    
    TEST(KinectTest, GetDistanceForRobotMatrix8){
        //Arrange
        
        // Mock obstacle
        Vec2f obstacle1(0.814f, 1.168f);
        Vec2f obstacle2(0.268f, 1.868f);

        
        Mat testMatrix = Utility::readFromFile("matrixRobot8.xml");
        Kinect kinect(obstacle1, obstacle2);
        
        float valueX = 0.738f;
        float valueZ = 1.043f;
        
        //Act
        kinect.findRobot(testMatrix);
        Vec2f robot = kinect.getRobot();
        float positionX = robot[0];
        float positionZ = robot[1];
        
        
        //Assert
        ASSERT_TRUE(positionX >= valueX-0.01 && positionX < valueX + 0.01);
        ASSERT_TRUE(positionZ >= valueZ-0.01 && positionZ < valueZ + 0.01);
    }

    //TODO : Correct the robotPosition. The black border makes some error in median calculation
    TEST(KinectTest, GetDistanceForRobotMatrix9){
        //Arrange
        
        // Mock obstacle
        Vec2f obstacle1(0.302f, 1.13f);
        Vec2f obstacle2(0.26f, 1.86f);
        
        Mat testMatrix = Utility::readFromFile("matrixRobot9.xml");
        Kinect kinect(obstacle1, obstacle2);
        
        float valueX = 0.28f;
        float valueZ = 1.03f;
        
        //Act
        kinect.findRobot(testMatrix);
        Vec2f robot = kinect.getRobot();
        float positionX = robot[0];
        float positionZ = robot[1];
        
        
        //Assert
        ASSERT_TRUE(positionX >= valueX-0.01 && positionX < valueX + 0.01);
        ASSERT_TRUE(positionZ >= valueZ-0.01 && positionZ < valueZ + 0.01);
    }
    
    TEST(KinectTest, GetDistanceForObstacleMatrix1){
        //Arrange
        Mat testMatrix = Utility::readFromFile("matrixRobot1.xml");
        Kinect kinect;
        
        float valueX1 = 0.46f;
        float valueX2 = 0;
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
        ASSERT_TRUE(positionX1 >= valueX1-0.01 && positionX1 < valueX1 + 0.01);
        ASSERT_TRUE(positionZ1 >= valueZ1-0.01 && positionZ1 < valueZ1 + 0.01);
    
        ASSERT_TRUE(positionX2 == valueX2 && positionX2 == valueX2);
        ASSERT_TRUE(positionZ2 == valueZ2 && positionZ2 == valueZ2);
    }
    
    TEST(KinectTest, GetDistanceForObstacleMatrix2){
        //Arrange
        Mat testMatrix = Utility::readFromFile("matrixRobot2.xml");
        Kinect kinect;
        
        float valueX1 = 0.742f;
        float valueX2 = 0.463f;
        float valueZ1 = 1.415f;
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
        ASSERT_TRUE(positionX1 >= valueX1-0.01 && positionX1 < valueX1 + 0.01);
        ASSERT_TRUE(positionZ1 >= valueZ1-0.01 && positionZ1 < valueZ1 + 0.01);
        
        ASSERT_TRUE(positionX2 >= valueX2-0.01 && positionX2 < valueX2 + 0.01);
        ASSERT_TRUE(positionZ2 >= valueZ2-0.01 && positionZ2 < valueZ2 + 0.01);
    }
    
    //Test qui doit trouver seulement 1 obstacle sachant que l'autre est dans la zone morte de la Kinect
    TEST(KinectTest, GetDistanceForObstacleMatrix3){
        //Arrange
        Mat testMatrix = Utility::readFromFile("matrixRobot3.xml");
        Kinect kinect;
        
        float valueX1 = 0.74f;
        float valueX2 = 0;
        float valueZ1 = 1.404f;
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
        ASSERT_TRUE(positionX1 >= valueX1-0.01 && positionX1 < valueX1 + 0.01);
        ASSERT_TRUE(positionZ1 >= valueZ1-0.01 && positionZ1 < valueZ1 + 0.01);
        
        ASSERT_TRUE(positionX2 >= valueX2-0.01 && positionX2 < valueX2 + 0.01);
        ASSERT_TRUE(positionZ2 >= valueZ2-0.01 && positionZ2 < valueZ2 + 0.01);
    }


    //TODO : Check real distance, strange results
    TEST(KinectTest, GetDistanceForObstacleMatrix4){
        //Arrange
        Mat testMatrix = Utility::readFromFile("matrixRobot4.xml");
        Kinect kinect;
        
        float valueX1 = 0.815f;
        float valueX2 = 0.367f;
        float valueZ1 = 1.01f;
        float valueZ2 = 2.11f;
        
        //Act
        kinect.findCenteredObstacle(testMatrix);
        Vec2f obstacle1 = kinect.getObstacle1();
        float positionX1 = obstacle1[0];
        float positionZ1 = obstacle1[1];
        Vec2f obstacle2 = kinect.getObstacle2();
        float positionX2 = obstacle2[0];
        float positionZ2 = obstacle2[1];
        
        //Assert
        ASSERT_TRUE(positionX1 >= valueX1-0.01 && positionX1 < valueX1 + 0.01);
        ASSERT_TRUE(positionZ1 >= valueZ1-0.01 && positionZ1 < valueZ1 + 0.01);
        
        ASSERT_TRUE(positionX2 >= valueX2-0.01 && positionX2 < valueX2 + 0.01);
        ASSERT_TRUE(positionZ2 >= valueZ2-0.01 && positionZ2 < valueZ2 + 0.01);
    }
    
    TEST(KinectTest, GetDistanceForObstacleMatrix5){
        //Arrange
        Mat testMatrix = Utility::readFromFile("matrixRobot5.xml");
        Kinect kinect;
        
        float valueX1 = 0.602f;
        float valueX2 = 0.468f;
        float valueZ1 = 1.164f;
        float valueZ2 = 1.167f;
        
        //Act
        kinect.findCenteredObstacle(testMatrix);
        Vec2f obstacle1 = kinect.getObstacle1();
        float positionX1 = obstacle1[0];
        float positionZ1 = obstacle1[1];
        Vec2f obstacle2 = kinect.getObstacle2();
        float positionX2 = obstacle2[0];
        float positionZ2 = obstacle2[1];
        
        //Assert
        ASSERT_TRUE(positionX1 >= valueX1-0.01 && positionX1 < valueX1 + 0.01);
        ASSERT_TRUE(positionZ1 >= valueZ1-0.01 && positionZ1 < valueZ1 + 0.01);
        
        ASSERT_TRUE(positionX2 >= valueX2-0.01 && positionX2 < valueX2 + 0.01);
        ASSERT_TRUE(positionZ2 >= valueZ2-0.01 && positionZ2 < valueZ2 + 0.01);
    }
    
    TEST(KinectTest, GetDistanceForObstacleMatrix6){
        //Arrange
        Mat testMatrix = Utility::readFromFile("matrixRobot6.xml");
        Kinect kinect;
        
        float valueX1 = 0.607f;
        float valueX2 = 0.421f;
        float valueZ1 = 1.161f;
        float valueZ2 = 1.046f;
        
        //Act
        kinect.findCenteredObstacle(testMatrix);
        Vec2f obstacle1 = kinect.getObstacle1();
        float positionX1 = obstacle1[0];
        float positionZ1 = obstacle1[1];
        Vec2f obstacle2 = kinect.getObstacle2();
        float positionX2 = obstacle2[0];
        float positionZ2 = obstacle2[1];
        
        //Assert
        ASSERT_TRUE(positionX1 >= valueX1-0.01 && positionX1 < valueX1 + 0.01);
        ASSERT_TRUE(positionZ1 >= valueZ1-0.01 && positionZ1 < valueZ1 + 0.01);
        
        ASSERT_TRUE(positionX2 >= valueX2-0.01 && positionX2 < valueX2 + 0.01);
        ASSERT_TRUE(positionZ2 >= valueZ2-0.01 && positionZ2 < valueZ2 + 0.01);
    }
    
    TEST(KinectTest, GetDistanceForObstacleMatrix7){
        //Arrange
        Mat testMatrix = Utility::readFromFile("matrixRobot7.xml");
        Kinect kinect;
        
        float valueX1 = 0.583f;
        float valueX2 = 0.422f;
        float valueZ1 = 1.164f;
        float valueZ2 = 1.04f;
        
        //Act
        kinect.findCenteredObstacle(testMatrix);
        Vec2f obstacle1 = kinect.getObstacle1();
        float positionX1 = obstacle1[0];
        float positionZ1 = obstacle1[1];
        Vec2f obstacle2 = kinect.getObstacle2();
        float positionX2 = obstacle2[0];
        float positionZ2 = obstacle2[1];
        
        //Assert
        ASSERT_TRUE(positionX1 >= valueX1-0.01 && positionX1 < valueX1 + 0.01);
        ASSERT_TRUE(positionZ1 >= valueZ1-0.01 && positionZ1 < valueZ1 + 0.01);
        
        ASSERT_TRUE(positionX2 >= valueX2-0.01 && positionX2 < valueX2 + 0.01);
        ASSERT_TRUE(positionZ2 >= valueZ2-0.01 && positionZ2 < valueZ2 + 0.01);
    }
    
    TEST(KinectTest, GetDistanceForObstacleMatrix8){
        //Arrange
        Mat testMatrix = Utility::readFromFile("matrixRobot8.xml");
        Kinect kinect;
        
        float valueX1 = 0.814f;
        float valueX2 = 0.268f;
        float valueZ1 = 1.168f;
        float valueZ2 = 1.868f;
        
        //Act
        kinect.findCenteredObstacle(testMatrix);
        Vec2f obstacle1 = kinect.getObstacle1();
        float positionX1 = obstacle1[0];
        float positionZ1 = obstacle1[1];
        Vec2f obstacle2 = kinect.getObstacle2();
        float positionX2 = obstacle2[0];
        float positionZ2 = obstacle2[1];
        
        //Assert
        ASSERT_TRUE(positionX1 >= valueX1-0.01 && positionX1 < valueX1 + 0.01);
        ASSERT_TRUE(positionZ1 >= valueZ1-0.01 && positionZ1 < valueZ1 + 0.01);
        
        ASSERT_TRUE(positionX2 >= valueX2-0.01 && positionX2 < valueX2 + 0.01);
        ASSERT_TRUE(positionZ2 >= valueZ2-0.01 && positionZ2 < valueZ2 + 0.01);
    }
    
    TEST(KinectTest, GetDistanceForObstacleMatrix9){
        //Arrange
        Mat testMatrix = Utility::readFromFile("matrixRobot9.xml");
        Kinect kinect;
        
        float valueX1 = 0.302f;
        float valueX2 = 0.26f;
        float valueZ1 = 1.129f;
        float valueZ2 = 1.859f;
        
        //Act
        kinect.findCenteredObstacle(testMatrix);
        Vec2f obstacle1 = kinect.getObstacle1();
        float positionX1 = obstacle1[0];
        float positionZ1 = obstacle1[1];
        Vec2f obstacle2 = kinect.getObstacle2();
        float positionX2 = obstacle2[0];
        float positionZ2 = obstacle2[1];
        
        //Assert
        ASSERT_TRUE(positionX1 >= valueX1-0.01 && positionX1 < valueX1 + 0.01);
        ASSERT_TRUE(positionZ1 >= valueZ1-0.01 && positionZ1 < valueZ1 + 0.01);
        
        ASSERT_TRUE(positionX2 >= valueX2-0.01 && positionX2 < valueX2 + 0.01);
        ASSERT_TRUE(positionZ2 >= valueZ2-0.01 && positionZ2 < valueZ2 + 0.01);
    }
    
    TEST(KinectTest, getTrueCoordFromKinectCoords){
        //Arrange
        Kinect kinect;
        Vec3f depthXYZ(1,2,3);
        
        //Act
        Vec2f trueCoords = kinect.getTrueCoordFromKinectCoord(depthXYZ);
        
        //Assert a range
        ASSERT_TRUE(trueCoords[0] >= (0.42 - 0.3) || trueCoords[0] <= (0.42 + 0.3));
        ASSERT_TRUE(trueCoords[1] >= (2.59 - 0.3) || trueCoords[1] <= (2.59 + 0.3));
    }

}

int main(int argc, char * argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

