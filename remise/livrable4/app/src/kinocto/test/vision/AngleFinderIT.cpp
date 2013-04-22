#include <gtest/gtest.h>

#include "vision/AngleFinder.h"

using namespace cv;
using namespace std;

namespace {
class AngleFinderIT: public ::testing::Test {
public:

protected:
    static const double PRECISION = 0.05;
    AngleFinder angleFinder;
};

TEST_F(AngleFinderIT, find1Degree) {
    Mat img = imread("img/testWallAngleFinder/1degree.png");

    double angle = angleFinder.findWallAngle2(img);

    ASSERT_NEAR(1.0, angle, PRECISION);
}

TEST_F(AngleFinderIT, findMinus1Degree) {
    Mat img = imread("img/testWallAngleFinder/moin1degree.png");

    double angle = angleFinder.findWallAngle2(img);

    ASSERT_NEAR(-1.0, angle, PRECISION);
}

TEST_F(AngleFinderIT, find3Degree) {
    Mat img = imread("img/testWallAngleFinder/3degree.png");

    double angle = angleFinder.findWallAngle2(img);

    ASSERT_NEAR(3, angle, PRECISION);
}

TEST_F(AngleFinderIT, findMinus3Degree) {
    Mat img = imread("img/testWallAngleFinder/moin3degree.png");

    double angle = angleFinder.findWallAngle2(img);

    ASSERT_NEAR(-3.0, angle, PRECISION);
}

TEST_F(AngleFinderIT, find6Degree) {
    Mat img = imread("img/testWallAngleFinder/6degree.png");

    double angle = angleFinder.findWallAngle2(img);

    ASSERT_NEAR(6, angle, PRECISION);
}

TEST_F(AngleFinderIT, findMinus6Degree) {
    Mat img = imread("img/testWallAngleFinder/moin6degree.png");

    double angle = angleFinder.findWallAngle2(img);

    ASSERT_NEAR(-6.0, angle, PRECISION);
}

TEST_F(AngleFinderIT, find15Degree) {
    Mat img = imread("img/testWallAngleFinder/15degree.png");

    double angle = angleFinder.findWallAngle2(img);

    ASSERT_NEAR(15.0, angle, PRECISION);
}

TEST_F(AngleFinderIT, findMinus15Degree) {
    Mat img = imread("img/testWallAngleFinder/moin15degree.png");

    double angle = angleFinder.findWallAngle2(img);

    ASSERT_NEAR(-15.0, angle, PRECISION);
}

TEST_F(AngleFinderIT, findDegreeOnMur1) {
    Mat img = imread("img/testWallAngleFinder/mur1.png");

    double angle = angleFinder.findWallAngle2(img);

    ASSERT_NEAR(-0.99, angle, PRECISION);
}

TEST_F(AngleFinderIT, findDeuxAngleGauche2) {
    Mat img = imread("img/testWallAngleFinder/deuxAnglesGauche.png");

    double angle = angleFinder.findWallAngle2(img);

    ASSERT_NEAR(3, angle, PRECISION);
}

TEST_F(AngleFinderIT, findDeuxAngleDroite2) {
    Mat img = imread("img/testWallAngleFinder/deuxAnglesDroite.png");

    double angle = angleFinder.findWallAngle2(img);

    ASSERT_NEAR(-3, angle, PRECISION);
}

}
