#include <gtest/gtest.h>

#include "vision/BlueCornerFinder.h"

using namespace cv;
using namespace std;

namespace {
class BlueCornerFinderIT: public ::testing::Test {
public:

protected:
    static const double PRECISION = 0.05;
    BlueCornerFinder blueCornerFinder;
};

TEST_F(BlueCornerFinderIT, returnZeroWhenNotAGoodImage) {
    Mat img = imread("img/testBlueCornerFinder/1.png");

    //double angle = blueCornerFinder.findWallAngle2(img);

    //ASSERT_NEAR(0.0, angle, PRECISION);
}

TEST_F(BlueCornerFinderIT, find1Degree) {
    Mat img = imread("img/testBlueCornerFinder/2.png");

    //double angle = blueCornerFinder.findWallAngle2(img);

    //ASSERT_NEAR(1.0, angle, PRECISION);
}

TEST_F(BlueCornerFinderIT, canFindTheBlueCorner) {
    Mat img = imread("img/testBlueCornerFinder/1.png");
    ASSERT_TRUE(blueCornerFinder.isPresent(img));
}

}
