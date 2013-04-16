#include <gtest/gtest.h>

#include "vision/AngleFinder.h"

using namespace cv;
using namespace std;

namespace {
class GreenBorderAngleFinderIT: public ::testing::Test {
public:

protected:
    static const double PRECISION = 0.05;
    AngleFinder angleFinder;
};

TEST_F(GreenBorderAngleFinderIT, returnRightAngle1) {
    Mat img = imread("img/testGreenBorder/1.png");

    double angle = angleFinder.findGreenBorderAngle(img);

    ASSERT_NEAR(1.2, angle, PRECISION);
}

TEST_F(GreenBorderAngleFinderIT, returnRightAngle2) {
    Mat img = imread("img/testGreenBorder/2.png");

    double angle = angleFinder.findGreenBorderAngle(img);

    ASSERT_NEAR(-3.6, angle, PRECISION);
}

TEST_F(GreenBorderAngleFinderIT, returnRightAngle3) {
    Mat img = imread("img/testGreenBorder/3.png");

    double angle = angleFinder.findGreenBorderAngle(img);

    ASSERT_NEAR(1.8, angle, PRECISION);
}

}
