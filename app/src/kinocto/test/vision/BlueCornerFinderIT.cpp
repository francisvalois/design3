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

TEST_F(BlueCornerFinderIT, test1) {
    Mat img = imread("img/testBlueCornerFinder/3.jpg");
    blueCornerFinder.isPresent(img);
}

TEST_F(BlueCornerFinderIT, test2) {
    Mat img = imread("img/testBlueCornerFinder/6.jpg");
    blueCornerFinder.isPresent(img);
}

}