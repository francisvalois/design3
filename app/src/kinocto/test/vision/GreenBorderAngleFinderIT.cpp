#include <gtest/gtest.h>

#include "vision/GreenBorderAngleFinder.h"

using namespace cv;
using namespace std;

namespace {
class GreenBorderAngleFinderIT: public ::testing::Test {
public:

protected:
    static const double PRECISION = 0.05;
    GreenBorderAngleFinder greenBorderAngleFinder;
};

TEST_F(GreenBorderAngleFinderIT, returnZeroWhenNotAGoodImage) {
    Mat img = imread("img/testGreenBorder/1.png");

    double angle = greenBorderAngleFinder.findAngle(img);

    ASSERT_NEAR(2.87, angle, PRECISION);
}

}
