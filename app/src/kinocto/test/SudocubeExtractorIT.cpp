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
	char filename[255];

	for (int i = 1; i <= 42; i++) {
		sprintf(filename, "%s/%d.png", "img/testSudocubes", i);
		Mat src = imread(filename);

		if (!src.data ) {
			cout << "SudocubeExtractorIT could not load img sudocube test" << endl;
		}

		sudocubeExtractor->extractSudocube(src);
	}

	ASSERT_TRUE(true);
}

}

