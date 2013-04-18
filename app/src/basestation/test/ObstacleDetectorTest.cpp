#include "gtest/gtest.h"
#include "opencv2/core/core.hpp"

#include "KinectUtility.h"
#include "ObstaclesDetector.h"
#include "opencv2/highgui/highgui.hpp"
#include "KinectTransformator.h"

using namespace cv;
using namespace std;

namespace {

    Mat testMatrix;
    Mat testRGB;


    void onMouse(int event, int x, int y, int flags, void *) {
        if (event == CV_EVENT_LBUTTONUP) {
            Vec3f s = testMatrix.at<Vec3f>(y, x);
            Vec2f realPosition = KinectTransformator::getTrueCoordFromKinectCoord(s);
            cout << "Pixel X :" << x << "Pixel Y :" << y << endl;
            cout << "Position X :" << realPosition[0] << " Position Y :" << s[1] << " Position Z:" << realPosition[1] << endl;
            cout << "From Kinect : Position X :" << s[0] << " Position Y :" << s[1] << " Position Z:" << s[2] << endl;
        }
        if (event == CV_EVENT_RBUTTONUP) {
            Vec3f s = testRGB.at<Vec3b>(x, y);
            cout << (int)s[0] << " " << (int)s[1] << " " << (int)s[2] << endl;

            cout << "Pixel X :" << x << "Pixel Y :" << y << endl;
        }
    }

    class ObstaclesDetectorTest : public ::testing::Test {
    protected:
        virtual void SetUp() {

            namedWindow("depth", 1);
            namedWindow("chess8", 1);
            setMouseCallback("depth", onMouse, 0);
            setMouseCallback("chess8", onMouse, 0);

            //KinectTransformator::setKinectAngle(0.555f);
//            Vec2f kinectPosition(0.10f, -0.44f);
//            KinectTransformator::setKinectPosition(kinectPosition);


        }

        ObstaclesDetector kinect;
    };

    TEST_F(ObstaclesDetectorTest, GetDistanceForObstacleMatrix1Table2){
        //Arrange
        testMatrix = Utility::readFromFile("table2/matrice2.xml");
        testRGB = imread("table2/rgb2.jpg");
        float valueZ2 = 2.25f;
        float valueZ1 = 2.25f;
        float valueX2 = 0.065f;
        float valueX1 = 1.06f;
        
        //Act
        kinect.findCenteredObstacle(testMatrix);
        Vec2f obstacle1 = kinect.getObstacle1();
        float positionX1 = obstacle1[0];
        float positionZ1 = obstacle1[1];
        Vec2f obstacle2 = kinect.getObstacle2();
        float positionX2 = obstacle2[0];
        float positionZ2 = obstacle2[1];

        cout << obstacle1 << endl;
        cout << obstacle2 << endl;
        
        //Assert
        ASSERT_TRUE(positionX1 >= valueX1 - 0.05 && positionX1 <= valueX1 + 0.05);
        ASSERT_TRUE(positionZ1 >= valueZ1 - 0.05 && positionZ1 <= valueZ1 + 0.05);
    
        ASSERT_TRUE(positionX2 >= valueX2 - 0.05 && positionX2 <= valueX2 + 0.05);
        ASSERT_TRUE(positionZ2 >= valueZ2 - 0.05 && positionZ2 <= valueZ2 + 0.05);
    }

    TEST_F(ObstaclesDetectorTest, GetDistanceForObstacleMatrix2Table2){
    //Arrange
    testMatrix = Utility::readFromFile("table2/matrice4.xml");
    testRGB = imread("table2/rgb4.jpg");

    float valueZ2 = 1.51f;
    float valueZ1 = 1.51f;
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

    cout << obstacle1 << endl;
    cout << obstacle2 << endl;

    //Assert
    ASSERT_TRUE(positionX1 >= valueX1 - 0.04 && positionX1 <= valueX1 + 0.04);
    ASSERT_TRUE(positionZ1 >= valueZ1 - 0.04 && positionZ1 <= valueZ1 + 0.04);

    ASSERT_TRUE(positionX2 >= valueX2 - 0.04 && positionX2 <= valueX2 + 0.04);
    ASSERT_TRUE(positionZ2 >= valueZ2 - 0.04 && positionZ2 <= valueZ2 + 0.04);
}

    TEST_F(ObstaclesDetectorTest, GetDistanceForObstacleMatrix3Table2){
    //Arrange
    testMatrix = Utility::readFromFile("table2/matrice5.xml");
    testRGB = imread("table2/rgb5.jpg");

    float valueZ2 = 1.175f;
    float valueZ1 = 1.175f;
    float valueX2 = 0.224f;
    float valueX1 = 0.88f;

    //Act
    kinect.findCenteredObstacle(testMatrix);
    Vec2f obstacle1 = kinect.getObstacle1();
    float positionX1 = obstacle1[0];
    float positionZ1 = obstacle1[1];
    Vec2f obstacle2 = kinect.getObstacle2();
    float positionX2 = obstacle2[0];
    float positionZ2 = obstacle2[1];
    cout << obstacle1 << endl;
    cout << obstacle2 << endl;

    //Assert
    ASSERT_TRUE(positionX1 >= valueX1 - 0.03 && positionX1 <= valueX1 + 0.03);
    ASSERT_TRUE(positionZ1 >= valueZ1 - 0.03 && positionZ1 <= valueZ1 + 0.03);

    ASSERT_TRUE(positionX2 >= valueX2 - 0.03 && positionX2 <= valueX2 + 0.03);
    ASSERT_TRUE(positionZ2 >= valueZ2 - 0.03 && positionZ2 <= valueZ2 + 0.03);
}

    TEST_F(ObstaclesDetectorTest, GetDistanceForObstacleMatrix1Table1){
    //Arrange
    testMatrix = Utility::readFromFile("table1/matrice2.xml");
    testRGB = imread("table1/rgb2.jpg");
    float valueZ2 = 2.25f;
    float valueZ1 = 2.25f;
    float valueX2 = 0.065f;
    float valueX1 = 1.06f;

    //Act
    kinect.findCenteredObstacle(testMatrix);
    Vec2f obstacle1 = kinect.getObstacle1();
    float positionX1 = obstacle1[0];
    float positionZ1 = obstacle1[1];
    Vec2f obstacle2 = kinect.getObstacle2();
    float positionX2 = obstacle2[0];
    float positionZ2 = obstacle2[1];

    cout << obstacle1 << endl;
    cout << obstacle2 << endl;

    //Assert
    ASSERT_TRUE(positionX1 >= valueX1 - 0.05 && positionX1 <= valueX1 + 0.05);
    ASSERT_TRUE(positionZ1 >= valueZ1 - 0.05 && positionZ1 <= valueZ1 + 0.05);

    ASSERT_TRUE(positionX2 >= valueX2 - 0.05 && positionX2 <= valueX2 + 0.05);
    ASSERT_TRUE(positionZ2 >= valueZ2 - 0.05 && positionZ2 <= valueZ2 + 0.05);
}

    TEST_F(ObstaclesDetectorTest, GetDistanceForObstacleMatrix2Table1){
    //Arrange
    testMatrix = Utility::readFromFile("table1/matrice3.xml");
    testRGB = imread("table1/rgb3.jpg");

    float valueZ2 = 1.51f;
    float valueZ1 = 1.51f;
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

    cout << obstacle1 << endl;
    cout << obstacle2 << endl;

    //Assert
    ASSERT_TRUE(positionX1 >= valueX1 - 0.04 && positionX1 <= valueX1 + 0.04);
    ASSERT_TRUE(positionZ1 >= valueZ1 - 0.04 && positionZ1 <= valueZ1 + 0.04);

    ASSERT_TRUE(positionX2 >= valueX2 - 0.04 && positionX2 <= valueX2 + 0.04);
    ASSERT_TRUE(positionZ2 >= valueZ2 - 0.04 && positionZ2 <= valueZ2 + 0.04);
}

    TEST_F(ObstaclesDetectorTest, GetDistanceForObstacleMatrix3Table1){
    //Arrange
    testMatrix = Utility::readFromFile("table1/matrice5.xml");
    testRGB = imread("table1/rgb5.jpg");

    float valueZ2 = 1.175f;
    float valueZ1 = 1.175f;
    float valueX2 = 0.224f;
    float valueX1 = 0.88f;

    //Act
    kinect.findCenteredObstacle(testMatrix);
    Vec2f obstacle1 = kinect.getObstacle1();
    float positionX1 = obstacle1[0];
    float positionZ1 = obstacle1[1];
    Vec2f obstacle2 = kinect.getObstacle2();
    float positionX2 = obstacle2[0];
    float positionZ2 = obstacle2[1];
    cout << obstacle1 << endl;
    cout << obstacle2 << endl;

    //Assert
    ASSERT_TRUE(positionX1 >= valueX1 - 0.03 && positionX1 <= valueX1 + 0.03);
    ASSERT_TRUE(positionZ1 >= valueZ1 - 0.03 && positionZ1 <= valueZ1 + 0.03);

    ASSERT_TRUE(positionX2 >= valueX2 - 0.03 && positionX2 <= valueX2 + 0.03);
    ASSERT_TRUE(positionZ2 >= valueZ2 - 0.03 && positionZ2 <= valueZ2 + 0.03);
}


}

