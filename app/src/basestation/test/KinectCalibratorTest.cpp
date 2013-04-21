#include "gtest/gtest.h"
#include "opencv2/core/core.hpp"

#include "KinectUtility.h"
#include "KinectTransformator.h"
#include "KinectCalibrator.h"

using namespace cv;
using namespace std;

namespace {
    class KinectCalibratorTest : public ::testing::Test {
    protected:
        virtual void SetUp() {
            KinectTransformator::setKinectAngle(0.3926f);
            Vec2f kinectPosition(0.10f, -0.44f);
            KinectTransformator::setKinectPosition(kinectPosition);
        }

        KinectCalibrator calibrator;
    };

    class KinectTransformatorMock: public KinectTransformator{
        public:
            float static getKinectAngle(){
                return _kinectAngleRad;
            }

            Vec2f static getKinectPosition(){
                return _kinectPosition;
            }
    };

    TEST_F(KinectCalibratorTest, calibrateTest1){
        //Arrange
        Mat testMatrix = Utility::readFromFile("img/testKinect/calibration1.xml");
        Mat rgbMatrix = imread("img/testKinect/calibration1.jpg");
        float expectedPositionX = 0.21f;
        float expectedPositionZ = -0.54f;
        float expectedAngle = 0.34f;

        //Act
        bool calibrationCompleted = calibrator.calibrate(rgbMatrix, testMatrix);
        float kinectAngle = KinectTransformatorMock::getKinectAngle();
        Vec2f kinectPosition = KinectTransformatorMock::getKinectPosition();

        //Assert
        ASSERT_TRUE(kinectAngle >= expectedAngle - 0.03 && kinectAngle <= expectedAngle + 0.03);

        ASSERT_TRUE(calibrationCompleted);

        ASSERT_TRUE(kinectPosition[0] >= expectedPositionX - 0.03 && kinectPosition[0] <= expectedPositionX + 0.03);
        ASSERT_TRUE(kinectPosition[1] >= expectedPositionZ - 0.03 && kinectPosition[1] <= expectedPositionZ + 0.03);
    }

    TEST_F(KinectCalibratorTest, calibrateTest2){
        //Arrange
        Mat testMatrix = Utility::readFromFile("img/testKinect/calibration2.xml");
        Mat rgbMatrix = imread("img/testKinect/calibration2.jpg");
        float expectedPositionX = 0.20f;
        float expectedPositionZ = -0.54f;
        float expectedAngle = 0.43f;

        //Act
        bool calibrationCompleted = calibrator.calibrate(rgbMatrix, testMatrix);
        float kinectAngle = KinectTransformatorMock::getKinectAngle();
        Vec2f kinectPosition = KinectTransformatorMock::getKinectPosition();

        //Assert
        ASSERT_TRUE(kinectAngle >= expectedAngle - 0.03 && kinectAngle <= expectedAngle + 0.03);

        ASSERT_TRUE(calibrationCompleted);

        ASSERT_TRUE(kinectPosition[0] >= expectedPositionX - 0.03 && kinectPosition[0] <= expectedPositionX + 0.03);
        ASSERT_TRUE(kinectPosition[1] >= expectedPositionZ - 0.03 && kinectPosition[1] <= expectedPositionZ + 0.03);
    }

    TEST_F(KinectCalibratorTest, calibrateTest3){
        //Arrange
        Mat testMatrix = Utility::readFromFile("img/testKinect/calibration3.xml");
        Mat rgbMatrix = imread("img/testKinect/calibration3.jpg");
        float expectedPositionX = 0.21f;
        float expectedPositionZ = -0.54f;
        float expectedAngle = 0.29f;

        //Act
        bool calibrationCompleted = calibrator.calibrate(rgbMatrix, testMatrix);
        float kinectAngle = KinectTransformatorMock::getKinectAngle();
        Vec2f kinectPosition = KinectTransformatorMock::getKinectPosition();

        //Assert
        ASSERT_TRUE(kinectAngle >= expectedAngle - 0.03 && kinectAngle <= expectedAngle + 0.03);

        ASSERT_TRUE(calibrationCompleted);

        ASSERT_TRUE(kinectPosition[0] >= expectedPositionX - 0.03 && kinectPosition[0] <= expectedPositionX + 0.03);
        ASSERT_TRUE(kinectPosition[1] >= expectedPositionZ - 0.03 && kinectPosition[1] <= expectedPositionZ + 0.03);
    }

}
