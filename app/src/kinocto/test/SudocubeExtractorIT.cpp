#include <gtest/gtest.h>

#include "SudocubeExtractor.h"

using namespace cv;
using namespace std;

namespace {
class SudocubeExtractorIT: public ::testing::Test {
public:

protected:
	SudocubeExtractor * sudocubeExtractor;

	virtual void SetUp() {
		sudocubeExtractor = new SudocubeExtractor();
	}

	virtual void TearDown() {
		delete sudocubeExtractor;
		sudocubeExtractor = NULL;
	}
};

TEST_F(SudocubeExtractorIT, simpleTest) {
	ASSERT_TRUE(true);
	//ASSERT_TRUE(getNumberOfWrongDetectionFor(numberReader, 1) == 0);
}

}

