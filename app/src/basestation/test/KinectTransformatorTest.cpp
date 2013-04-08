#include "gtest/gtest.h"
#include "opencv2/core/core.hpp"
#include "KinectTransformator.h"

using namespace cv;
using namespace std;

namespace { 
    class KinectTransformatorTest : public ::testing::Test {
    protected:
        virtual void SetUp() {
            KinectTransformator::setKinectAngle(22.5);
            Vec2f kinectPosition(0.10f, -0.44f);
            KinectTransformator::setKinectPosition(kinectPosition);
        }
    };

    TEST_F(KinectTransformatorTest, getRotatedXZCoordFromKinectCoordTest){
        //Arrange
        float valueX = 0.15f;
        float valueZ = 2.23f; 
        float trueValueX = 0.7148f;
        float trueValueZ = 2.1176f; 

        Vec3f value(valueX, 0, valueZ);
        
        //Act
        Vec2f rotatedPosition = KinectTransformator::getRotatedXZCoordFromKinectCoord(value);
        float rotatedPositionX = rotatedPosition[0];
        float rotatedPositionZ = rotatedPosition[1];
        
        //Assert
        ASSERT_TRUE(rotatedPositionX >= trueValueX - 0.01 && rotatedPositionX <= trueValueX + 0.01);
        ASSERT_TRUE(rotatedPositionZ >= trueValueZ - 0.01 && rotatedPositionZ <= trueValueZ + 0.01);
    }

    TEST_F(KinectTransformatorTest, translateXZCoordtoOrigin){
        //Arrange
        float valueX = 0.27f;
        float valueZ = 2.23f; 
        float trueValueX = 0.37f;
        float trueValueZ = 1.79f; 

        Vec2f value(valueX, valueZ);

        //Act
        Vec2f translatedPosition = KinectTransformator::translateXZCoordtoOrigin(value);

        //Assert
        ASSERT_TRUE(translatedPosition[0] >= trueValueX - 0.01 && translatedPosition[0] <= trueValueX + 0.01);
        ASSERT_TRUE(translatedPosition[1] >= trueValueZ - 0.01 && translatedPosition[1] <= trueValueZ + 0.01);
    }

    TEST_F(KinectTransformatorTest, getTrueCoordFromKinectCoord){
        //Arrange
        float valueX = 0.15f;
        float valueZ = 2.23f; 
        float trueValueX = 0.8148f;
        float trueValueZ = 1.6776; 

        Vec3f value(valueX, 0, valueZ);
    
        //Act
        Vec2f truePosition = KinectTransformator::getTrueCoordFromKinectCoord(value);

        //Assert
        ASSERT_TRUE(truePosition[0] >= trueValueX - 0.01 && truePosition[0] <= trueValueX + 0.01);
        ASSERT_TRUE(truePosition[1] >= trueValueZ - 0.01 && truePosition[1] <= trueValueZ + 0.01);
    }
}

