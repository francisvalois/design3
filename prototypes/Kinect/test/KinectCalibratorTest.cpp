#include "gtest/gtest.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "KinectUtility.h"
#include "ObstaclesDetector.h"
#include "KinectTransformator.h"
#include "KinectCalibrator.h"

using namespace cv;
using namespace std;

namespace { 
    class KinectCalibratorTest : public ::testing::Test {
    protected:
        virtual void SetUp() {
            KinectTransformator::setKinectAngle(22.5);
            Vec2f kinectPosition(0.10f, -0.44f);
            KinectTransformator::setKinectPosition(kinectPosition);
        }

        KinectCalibrator calibrator;
    };


    //TODO : Get real tests images. These ones only works with the old calibration algorithm
    TEST_F(KinectCalibratorTest, calibrateTest1){
        //Arrange
        Mat testMatrix = Utility::readFromFile("C:/Users/Francis/Documents/Visual Studio 2012/Projects/opencv/Debug/donnees/calibration1.xml");
        Mat rgbMatrix = imread("C:/Users/Francis/Documents/Visual Studio 2012/Projects/opencv/Debug/donnees/calibration1.jpg");
        float expectedPositionX = 0.10f;
        float expectedPositionZ = -0.45f; 
        float expectedAngle = 0.35f;
        
        //Act
        bool calibrationCompleted = calibrator.calibrate(rgbMatrix, testMatrix);
        float kinectAngle = KinectTransformator::getKinectAngle();
        Vec2f kinectPosition = KinectTransformator::getKinectPosition();
        
        //Assert
        ASSERT_TRUE(kinectAngle >= expectedAngle - 0.03 && kinectAngle <= expectedAngle + 0.03);
        
        ASSERT_TRUE(calibrationCompleted);

        ASSERT_TRUE(kinectPosition[0] >= expectedPositionX - 0.03 && kinectPosition[0] <= expectedPositionX + 0.03);
        ASSERT_TRUE(kinectPosition[1] >= expectedPositionZ - 0.03 && kinectPosition[1] <= expectedPositionZ + 0.03);
    }

    TEST_F(KinectCalibratorTest, calibrateTest2){
        //Arrange
        Mat testMatrix = Utility::readFromFile("C:/Users/Francis/Documents/Visual Studio 2012/Projects/opencv/Debug/donnees/calibration2.xml");
        Mat rgbMatrix = imread("C:/Users/Francis/Documents/Visual Studio 2012/Projects/opencv/Debug/donnees/calibration2.jpg");
        float expectedPositionX = 0.10f;
        float expectedPositionZ = -0.45f; 
        float expectedAngle = 0.35f;

        //Act
        bool calibrationCompleted = calibrator.calibrate(rgbMatrix, testMatrix);
        float kinectAngle = KinectTransformator::getKinectAngle();
        Vec2f kinectPosition = KinectTransformator::getKinectPosition();

        //Assert
        ASSERT_TRUE(kinectAngle >= expectedAngle - 0.03 && kinectAngle <= expectedAngle + 0.03);

        ASSERT_TRUE(calibrationCompleted);

        ASSERT_TRUE(kinectPosition[0] >= expectedPositionX - 0.03 && kinectPosition[0] <= expectedPositionX + 0.03);
        ASSERT_TRUE(kinectPosition[1] >= expectedPositionZ - 0.03 && kinectPosition[1] <= expectedPositionZ + 0.03);
    }

    TEST_F(KinectCalibratorTest, calibrateTest3){
        //Arrange
        Mat testMatrix = Utility::readFromFile("C:/Users/Francis/Documents/Visual Studio 2012/Projects/opencv/Debug/donnees/calibration3.xml");
        Mat rgbMatrix = imread("C:/Users/Francis/Documents/Visual Studio 2012/Projects/opencv/Debug/donnees/calibration3.jpg");
        float expectedPositionX = 0.10f;
        float expectedPositionZ = -0.45f; 
        float expectedAngle = 0.35f;

        //Act
        bool calibrationCompleted = calibrator.calibrate(rgbMatrix, testMatrix);
        float kinectAngle = KinectTransformator::getKinectAngle();
        Vec2f kinectPosition = KinectTransformator::getKinectPosition();

        //Assert
        ASSERT_TRUE(kinectAngle >= expectedAngle - 0.03 && kinectAngle <= expectedAngle + 0.03);

        ASSERT_TRUE(calibrationCompleted);

        ASSERT_TRUE(kinectPosition[0] >= expectedPositionX - 0.03 && kinectPosition[0] <= expectedPositionX + 0.03);
        ASSERT_TRUE(kinectPosition[1] >= expectedPositionZ - 0.03 && kinectPosition[1] <= expectedPositionZ + 0.03);
    }
 
}
