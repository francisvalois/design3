#include <gtest/gtest.h>

#include "vision/WallAngleFinder.h"

using namespace cv;
using namespace std;

namespace {
class WallAngleFinderIT: public ::testing::Test {
public:

protected:
    static const double PRECISION = 0.05;
    WallAngleFinder wallAngleFinder;
};

TEST_F(WallAngleFinderIT, returnZeroWhenNotAGoodImage) {
    Mat img = imread("img/testWallAngleFinder/empty.png");

    double angle = wallAngleFinder.findAngle(img);

    ASSERT_NEAR(0.0, angle, PRECISION);
}

TEST_F(WallAngleFinderIT, find1Degree) {
    Mat img = imread("img/testWallAngleFinder/1degree.png");

    double angle = wallAngleFinder.findAngle(img);

    ASSERT_NEAR(1.0, angle, PRECISION);
}


TEST_F(WallAngleFinderIT, findMinus1Degree) {
    Mat img = imread("img/testWallAngleFinder/moin1degree.png");

    double angle = wallAngleFinder.findAngle(img);

    ASSERT_NEAR(-1.0, angle, PRECISION);
}

TEST_F(WallAngleFinderIT, find3Degree) {
    Mat img = imread("img/testWallAngleFinder/3degree.png");

    double angle = wallAngleFinder.findAngle(img);

    ASSERT_NEAR(3, angle, PRECISION);
}

TEST_F(WallAngleFinderIT, findMinus3Degree) {
    Mat img = imread("img/testWallAngleFinder/moin3degree.png");

    double angle = wallAngleFinder.findAngle(img);

    ASSERT_NEAR(-3.0, angle, PRECISION);
}

TEST_F(WallAngleFinderIT, find6Degree) {
    Mat img = imread("img/testWallAngleFinder/6degree.png");

    double angle = wallAngleFinder.findAngle(img);

    ASSERT_NEAR(6, angle, PRECISION);
}

TEST_F(WallAngleFinderIT, findMinus6Degree) {
    Mat img = imread("img/testWallAngleFinder/moin6degree.png");

    double angle = wallAngleFinder.findAngle(img);

    ASSERT_NEAR(-6.0, angle, PRECISION);
}

TEST_F(WallAngleFinderIT, find15Degree) {
    Mat img = imread("img/testWallAngleFinder/15degree.png");

    double angle = wallAngleFinder.findAngle(img);

    ASSERT_NEAR(15.0, angle, PRECISION);
}

TEST_F(WallAngleFinderIT, findMinus15Degree) {
    Mat img = imread("img/testWallAngleFinder/moin15degree.png");

    double angle = wallAngleFinder.findAngle(img);

    ASSERT_NEAR(-15.0, angle, PRECISION);
}

TEST_F(WallAngleFinderIT, findDegreeOnMur1) {
    Mat img = imread("img/testWallAngleFinder/mur1.png");

    double angle = wallAngleFinder.findAngle(img);

    ASSERT_NEAR(-1.40, angle, PRECISION);
}

TEST_F(WallAngleFinderIT, findDegreeOnMur2) {
    Mat img = imread("img/testWallAngleFinder/mur2.png");

    double angle = wallAngleFinder.findAngle(img);

    ASSERT_NEAR(2.20, angle, PRECISION);
}

}
