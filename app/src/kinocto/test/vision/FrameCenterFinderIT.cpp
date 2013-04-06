#include <gtest/gtest.h>

#include "vision/FrameCenterFinder.h"

using namespace cv;
using namespace std;

namespace {
class FrameCenterFinderIT: public ::testing::Test {
public:

protected:
    static const double PRECISION = 0.05;
    FrameCenterFinder frameCenterFinder;
};

TEST_F(FrameCenterFinderIT, findCenterFrame1) {
    Mat img = imread("img/testFrameCenterFinder/1.png");
    double translateX = frameCenterFinder.getXTranslation(img);

    ASSERT_NEAR(-2.2, translateX, PRECISION);
}

TEST_F(FrameCenterFinderIT, findCenterFrame2) {
    Mat img = imread("img/testFrameCenterFinder/2.png");
    double translateX = frameCenterFinder.getXTranslation(img);

    ASSERT_NEAR(2.59, translateX, PRECISION);
}

TEST_F(FrameCenterFinderIT, findCenterFrame3) {
    Mat img = imread("img/testFrameCenterFinder/3.png");
    double translateX = frameCenterFinder.getXTranslation(img);

    ASSERT_NEAR(6.1, translateX, PRECISION);
}

TEST_F(FrameCenterFinderIT, findCenterFrame4) {
    Mat img = imread("img/testFrameCenterFinder/4.png");
    double translateX = frameCenterFinder.getXTranslation(img);

    ASSERT_NEAR(-3.67, translateX, PRECISION);
}

TEST_F(FrameCenterFinderIT, findCenterFrame5) {
    Mat img = imread("img/testFrameCenterFinder/5.png");
    double translateX = frameCenterFinder.getXTranslation(img);

    ASSERT_NEAR(2.2, translateX, PRECISION);
}

TEST_F(FrameCenterFinderIT, findCenterFrame6) {
    Mat img = imread("img/testFrameCenterFinder/6.png");
    double translateX = frameCenterFinder.getXTranslation(img);

    ASSERT_NEAR(-1.5, translateX, PRECISION);
}
}
