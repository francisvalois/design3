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

		if (numberReader->identifyNumber(number) != no) {
			nbOfWrongNumber++;
		}

	}

	return nbOfWrongNumber;
}

TEST_F(NumberReaderIT, trainedDataAreValid) {
	for (int i = 1; i <= NumberReader::CLASSES; i++) {
		for (int j = 1; j < NumberReader::TRAIN_SAMPLES; j++) {
			char filename[255];
			sprintf(filename, "img/trainingNumbers/%d/%d-%d.png", i, i, j);

			Mat numberImg = imread(filename, 1);
			int number = numberReader->identifyNumber(numberImg);

			ASSERT_TRUE(number == i);
		}
	}
}

TEST_F(NumberReaderIT, allSampleAreCorrectlyIdentifiedForClassOne) {
	ASSERT_TRUE(getNumberOfWrongDetectionFor(numberReader, 1) == 0);
}

TEST_F(NumberReaderIT, allSampleAreCorrectlyIdentifiedForClassTwo) {

	ASSERT_TRUE(getNumberOfWrongDetectionFor(numberReader, 2) == 0);
}

TEST_F(NumberReaderIT, allSampleAreCorrectlyIdentifiedForClassThree) {

	ASSERT_TRUE(getNumberOfWrongDetectionFor(numberReader, 3) == 0);
}

TEST_F(NumberReaderIT, allSampleAreCorrectlyIdentifiedForClassFour) {

	ASSERT_TRUE(getNumberOfWrongDetectionFor(numberReader, 4) == 0);
}

TEST_F(NumberReaderIT, allSampleAreCorrectlyIdentifiedForClassFive) {

	ASSERT_TRUE(getNumberOfWrongDetectionFor(numberReader, 5) == 0);
}

TEST_F(NumberReaderIT, allSampleAreCorrectlyIdentifiedForClassSix) {

	ASSERT_TRUE(getNumberOfWrongDetectionFor(numberReader, 6) == 0);
}

TEST_F(NumberReaderIT, allSampleAreCorrectlyIdentifiedForClassSeven) {

	ASSERT_TRUE(getNumberOfWrongDetectionFor(numberReader, 7) == 0);
}

TEST_F(NumberReaderIT, allSampleAreCorrectlyIdentifiedForClassEight) {

	ASSERT_TRUE(getNumberOfWrongDetectionFor(numberReader, 8) == 0);
}

}

