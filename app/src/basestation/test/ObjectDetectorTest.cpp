#include "gtest/gtest.h"
#include "opencv2/core/core.hpp"

#include "KinectUtility.h"
#include "ObstaclesDetector.h"
#include "KinectTransformator.h"

using namespace cv;
using namespace std;

namespace { 
    class ObjectDetectorTest : public ::testing::Test {
    protected:
        virtual void SetUp() {
            KinectTransformator::setKinectAngle(0.3926f);
            Vec2f kinectPosition(0.10f, -0.44f);
            KinectTransformator::setKinectPosition(kinectPosition);
        }
        vector<Rect> outQuads;
        ObjectDetector detector;
    };

    TEST_F(ObjectDetectorTest, getGeneratedQuads1){
        //Arrange
        Mat rgbMatrix = imread("img/testKinect/robotdetection3.jpg");

        float expectedQuadsCount = 14;
        Rect quad0 = Rect(347, 256, 8, 10);
        Rect quad5 = Rect(360, 257, 8, 10);
        Rect quad10 = Rect(374, 239, 9, 11);
        Rect quad13 = Rect(382, 230, 8, 11);
        
        //Act
        int quadsCount = detector.generateQuads(rgbMatrix, outQuads);
        
        //Assert
        ASSERT_TRUE(quadsCount == expectedQuadsCount);
        ASSERT_TRUE(outQuads[0] == quad0);
        ASSERT_TRUE(outQuads[5] == quad5);
        ASSERT_TRUE(outQuads[10] == quad10);
        ASSERT_TRUE(outQuads[13] == quad13);
    }

    TEST_F(ObjectDetectorTest, getGeneratedQuads2){
        //Arrange
        Mat rgbMatrix = imread("img/testKinect/robotdetection4.jpg");

        float expectedQuadsCount = 12;
        Rect quad0 = Rect(368, 241, 9, 8);
        Rect quad5 = Rect(382, 242, 9, 8);
        Rect quad10 = Rect(396, 256, 8, 8);
        Rect quad11 = Rect(404, 236, 8, 8);

        //Act
        int quadsCount = detector.generateQuads(rgbMatrix, outQuads);

        //Assert
        ASSERT_TRUE(quadsCount == expectedQuadsCount);
        ASSERT_TRUE(outQuads[0] == quad0);
        ASSERT_TRUE(outQuads[5] == quad5);
        ASSERT_TRUE(outQuads[10] == quad10);
        ASSERT_TRUE(outQuads[11] == quad11);
    }

    TEST_F(ObjectDetectorTest, getGeneratedQuads3){
        //Arrange
        Mat rgbMatrix = imread("img/testKinect/robotdetection5.jpg");

        float expectedQuadsCount = 5;
        Rect quad0 = Rect(458, 241, 9, 11);
        Rect quad4 = Rect(474, 234, 9, 9);

        //Act
        int quadsCount = detector.generateQuads(rgbMatrix, outQuads);

        //Assert
        ASSERT_TRUE(quadsCount == expectedQuadsCount);
        ASSERT_TRUE(outQuads[0] == quad0);
        ASSERT_TRUE(outQuads[4] == quad4);
    }

    TEST_F(ObjectDetectorTest, getGeneratedQuads4){
        //Arrange
        Mat rgbMatrix = imread("img/testKinect/robotdetection6.jpg");

        float expectedQuadsCount = 9;
        Rect quad0 = Rect(464, 241, 9, 8);
        Rect quad5 = Rect(480, 248, 9, 10);
        Rect quad8 = Rect(488, 265, 9, 9);

        //Act
        int quadsCount = detector.generateQuads(rgbMatrix, outQuads);

        //Assert
        ASSERT_TRUE(quadsCount == expectedQuadsCount);
        ASSERT_TRUE(outQuads[0] == quad0);
        ASSERT_TRUE(outQuads[5] == quad5);
        ASSERT_TRUE(outQuads[8] == quad8);
    }
    
}

