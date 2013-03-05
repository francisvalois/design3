#include <gtest/gtest.h>

#include "NumberReader.h"

using namespace cv;
using namespace std;

namespace {
class NumberReaderIT: public ::testing::Test {
public:

protected:
	NumberReader * numberReader;

	virtual void SetUp() {
		numberReader = new NumberReader();
	}

	virtual void TearDown() {
		delete numberReader;
		numberReader = NULL;
	}
};

int getNumberOfWrongDetectionFor(NumberReader * numberReader, int no) {
	char filename[255];
	int nbOfWrongNumber = 0;

	for (int i = 1; i <= 60; i++) {
		sprintf(filename, "img/testNumbers/%d/%d-%d.png", no, no, i);
		Mat number = imread(filename);
		if (!number.data) {
			cout << "Can't find test image number" << endl;
		}

		cvtColor(number, number, COLOR_BGR2GRAY);
		if (numberReader->identifyNumber(number) != no) {
			nbOfWrongNumber++;
		}

	}

	return nbOfWrongNumber;
}

TEST_F(NumberReaderIT, verifyAllNumberOne) {
	ASSERT_TRUE(getNumberOfWrongDetectionFor(numberReader, 1) == 0);
}

TEST_F(NumberReaderIT, verifyAllNumberTwo) {

	ASSERT_TRUE(getNumberOfWrongDetectionFor(numberReader, 2) == 0);
}

TEST_F(NumberReaderIT, verifyAllNumberThree) {

	ASSERT_TRUE(getNumberOfWrongDetectionFor(numberReader, 3) == 0);
}

TEST_F(NumberReaderIT, verifyAllNumberFour) {

	ASSERT_TRUE(getNumberOfWrongDetectionFor(numberReader, 4) == 0);
}

TEST_F(NumberReaderIT, verifyAllNumberFive) {

	ASSERT_TRUE(getNumberOfWrongDetectionFor(numberReader, 5) == 0);
}

TEST_F(NumberReaderIT, verifyAllNumberSix) {

	ASSERT_TRUE(getNumberOfWrongDetectionFor(numberReader, 6) == 0);
}

TEST_F(NumberReaderIT, verifyAllNumberSeven) {

	ASSERT_TRUE(getNumberOfWrongDetectionFor(numberReader, 7) == 0);
}

TEST_F(NumberReaderIT, verifyAllNumberEight) {

	ASSERT_TRUE(getNumberOfWrongDetectionFor(numberReader, 8) == 0);
}

}

