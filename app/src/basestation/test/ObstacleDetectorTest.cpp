#include "gtest/gtest.h"
#include "opencv2/core/core.hpp"

#include "KinectUtility.h"
#include "ObstaclesDetector.h"
#include "KinectTransformator.h"

using namespace cv;
using namespace std;

namespace { 
    class ObstaclesDetectorTest : public ::testing::Test {
    protected:
        virtual void SetUp() {
            KinectTransformator::setKinectAngle(22.5);
            Vec2f kinectPosition(0.10f, -0.44f);
            KinectTransformator::setKinectPosition(kinectPosition);
        }

        ObstaclesDetector kinect;
    };

    TEST_F(ObstaclesDetectorTest, GetDistanceForObstacleMatrix1){
        //Arrange
        Mat testMatrix = Utility::readFromFile("img/testKinect/table2Obstacle1.xml");
        
        float valueZ2 = 2.06f;
        float valueZ1 = 2.05f; 
        float valueX2 = 0.17f;
        float valueX1 = 0.91f;
        
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
    
    TEST_F(ObstaclesDetectorTest, GetDistanceForObstacleMatrix2){
        //Arrange
        Mat testMatrix = Utility::readFromFile("img/testKinect/table2Obstacle2.xml");
        
        float valueX1 = 0.87f;
        float valueX2 = 0.335f;
        float valueZ1 = 1.61f;
        float valueZ2 = 1.80f;
        
        //Act
        kinect.findCenteredObstacle(testMatrix);
        Vec2f obstacle1 = kinect.getObstacle1();
        float positionX1 = obstacle1[0];
        float positionZ1 = obstacle1[1];
        Vec2f obstacle2 = kinect.getObstacle2();
        float positionX2 = obstacle2[0];
        float positionZ2 = obstacle2[1];
        
        //Assert
        ASSERT_TRUE(positionX1 >= valueX1-0.03 && positionX1 < valueX1 + 0.03);
        ASSERT_TRUE(positionZ1 >= valueZ1-0.03 && positionZ1 < valueZ1 + 0.03);
        
        ASSERT_TRUE(positionX2 >= valueX2-0.03 && positionX2 < valueX2 + 0.03);
        ASSERT_TRUE(positionZ2 >= valueZ2-0.03 && positionZ2 < valueZ2 + 0.03);
    }
    
    TEST_F(ObstaclesDetectorTest, GetDistanceForObstacleMatrix3){
        //Arrange
        Mat testMatrix = Utility::readFromFile("img/testKinect/table2Obstacle3.xml");
        
        float valueX1 = 0.62f;
        float valueX2 = 0.49f;
        float valueZ1 = 1.52f;
        float valueZ2 = 1.515f;
        
        //Act
        kinect.findCenteredObstacle(testMatrix);
        Vec2f obstacle1 = kinect.getObstacle1();
        float positionX1 = obstacle1[0];
        float positionZ1 = obstacle1[1];
        Vec2f obstacle2 = kinect.getObstacle2();
        float positionX2 = obstacle2[0];
        float positionZ2 = obstacle2[1];
        
        //Assert
        ASSERT_TRUE(positionX1 >= valueX1-0.03 && positionX1 < valueX1 + 0.03);
        ASSERT_TRUE(positionZ1 >= valueZ1-0.03 && positionZ1 < valueZ1 + 0.03);

        ASSERT_TRUE(positionX2 >= valueX2-0.03 && positionX2 < valueX2 + 0.03);
        ASSERT_TRUE(positionZ2 >= valueZ2-0.03 && positionZ2 < valueZ2 + 0.03);
    }


    //TODO : Find a way to detect the obstacle in the back. In that XML File, the distance map is stange
    TEST_F(ObstaclesDetectorTest, GetDistanceForObstacleMatrix4){
        //Arrange
        Mat testMatrix = Utility::readFromFile("img/testKinect/table2Obstacle4.xml");
        
        float valueX1 = 0.82f;
        float valueX2 = 0.27f;
        float valueZ1 = 2.23f;
        float valueZ2 = 1.255f;
        
        //Act
        kinect.findCenteredObstacle(testMatrix);
        Vec2f obstacle1 = kinect.getObstacle1();
        float positionX1 = obstacle1[0];
        float positionZ1 = obstacle1[1];
        Vec2f obstacle2 = kinect.getObstacle2();
        float positionX2 = obstacle2[0];
        float positionZ2 = obstacle2[1];
        
        //Assert
        ASSERT_TRUE(positionX1 >= valueX1-0.03 && positionX1 < valueX1 + 0.03);
        ASSERT_TRUE(positionZ1 >= valueZ1-0.03 && positionZ1 < valueZ1 + 0.03);
       
        ASSERT_TRUE(positionX2 >= valueX2-0.03 && positionX2 < valueX2 + 0.03);
        ASSERT_TRUE(positionZ2 >= valueZ2-0.03 && positionZ2 < valueZ2 + 0.03);
    }
}

