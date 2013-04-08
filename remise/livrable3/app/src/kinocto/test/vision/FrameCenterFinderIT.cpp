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
    Mat img = imread("img/testFrameCenterFinder/1.jpg");
    double translateX = frameCenterFinder.getXTranslation(img);

    ASSERT_NEAR(2.2, translateX, PRECISION);
}

TEST_F(FrameCenterFinderIT, findCenterFrame2) {
    Mat img = imread("img/testFrameCenterFinder/2.jpg");
    double translateX = frameCenterFinder.getXTranslation(img);

    ASSERT_NEAR(-2.55, translateX, PRECISION);
}

TEST_F(FrameCenterFinderIT, findCenterFrame3) {
    Mat img = imread("img/testFrameCenterFinder/3.jpg");
    double translateX = frameCenterFinder.getXTranslation(img);

    ASSERT_NEAR(-6.85, translateX, PRECISION);
}

TEST_F(FrameCenterFinderIT, findCenterFrame4) {
    Mat img = imread("img/testFrameCenterFinder/4.jpg");
    double translateX = frameCenterFinder.getXTranslation(img);

    ASSERT_NEAR(0.75, translateX, PRECISION);
}

TEST_F(FrameCenterFinderIT, findCenterFrame5) {
    Mat img = imread("img/testFrameCenterFinder/5.jpg");
    double translateX = frameCenterFinder.getXTranslation(img);

    ASSERT_NEAR(-5.6, translateX, PRECISION);
}

TEST_F(FrameCenterFinderIT, findCenterFrame6) {
    Mat img = imread("img/testFrameCenterFinder/6.jpg");
    double translateX = frameCenterFinder.getXTranslation(img);

    ASSERT_NEAR(1.75, translateX, PRECISION);
}
}
