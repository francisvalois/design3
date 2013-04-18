#include "gtest/gtest.h"
#include "opencv2/core/core.hpp"

#include "KinectUtility.h"
#include "RobotDetector.h"
#include "KinectTransformator.h"

using namespace cv;
using namespace std;

namespace {


    Mat testMatrix;
    Mat rgbMatrix;

    void onMouse(int event, int x, int y, int flags, void *) {
        if (event == CV_EVENT_LBUTTONUP) {
            Vec3f s = testMatrix.at<Vec3f>(y, x);
            Vec2f realPosition = KinectTransformator::getTrueCoordFromKinectCoord(s);
            cout << "Pixel X :" << x << "Pixel Y :" << y << endl;
            cout << "Position X :" << realPosition[0] << " Position Y :" << s[1] << " Position Z:" << realPosition[1] << endl;
            cout << "From Kinect : Position X :" << s[0] << " Position Y :" << s[1] << " Position Z:" << s[2] << endl;
        }
        if (event == CV_EVENT_RBUTTONUP) {
            Vec3f s = rgbMatrix.at<Vec3b>(x, y);
            cout << (int)s[0] << " " << (int)s[1] << " " << (int)s[2] << endl;

            cout << "Pixel X :" << x << "Pixel Y :" << y << endl;
        }
    }





    class RobotDetectorTest : public ::testing::Test {
    protected:
        virtual void SetUp() {
            namedWindow("depth", 1);
            namedWindow("chess8", 1);
            setMouseCallback("depth", onMouse, 0);
            setMouseCallback("chess8", onMouse, 0);
        }

        RobotDetector kinect;
    };

    TEST_F(RobotDetectorTest, GetDistanceForRobotMatrix1Table2){
        //Arrange
        testMatrix = Utility::readFromFile("table2/matrice1.xml");
        rgbMatrix = imread("table2/rgb1.jpg");
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

    TEST_F(RobotDetectorTest, GetDistanceForRobotMatrix2Table2){
        //Arrange
         testMatrix = Utility::readFromFile("table2/matrice3.xml");
         rgbMatrix = imread("table2/rgb3.jpg");
        float valueZ = 0.55f;
        float valueX = 0.555f;
        float angleValue = 0.38f;

        //Act
        kinect.findRobotWithAngle(testMatrix, rgbMatrix);
        Vec2f obstacle1 = kinect.getRobotPosition();
        float positionX = obstacle1[0];
        float positionZ = obstacle1[1];
        float robotAngle = kinect.getRobotAngle();

    imshow("chess8", rgbMatrix);
    imshow("depth", testMatrix);

        //Assert
        ASSERT_TRUE(positionX >= valueX - 0.03 && positionX <= valueX + 0.03);
        ASSERT_TRUE(positionZ >= valueZ - 0.03 && positionZ <= valueZ + 0.03);

        ASSERT_TRUE(robotAngle >= angleValue - 0.05 && robotAngle <= angleValue + 0.05);
    }

    TEST_F(RobotDetectorTest, GetDistanceForRobotMatrix1Table1){
        //Arrange
        testMatrix = Utility::readFromFile("table1/matrice1.xml");
        rgbMatrix = imread("table1/rgb1.jpg");
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

    TEST_F(RobotDetectorTest, GetDistanceForRobotMatrix2Table1){
        //Arrange
        testMatrix = Utility::readFromFile("table1/matrice4.xml");
        rgbMatrix = imread("table1/rgb4.jpg");
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
}

