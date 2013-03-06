#include <gtest/gtest.h>

#include "Case.h"

    const int MAXIMUM_NUMBER_OF_POSSIBILITIES = 8;
    const int A_VALID_SUDOKUBE_CASE_VALUE = 8;
    const int A_INVALID_SUDOKUBE_CASE_VALUE = 9;
    const int VALID_COORDINATE_I = 1;
    const int VALID_COORDINATE_J = 2;
    const int VALID_COORDINATE_K = 3;
    const int INVALID_COORDINATE = 7;

namespace {
  class CaseTest : public ::testing::Test {
  protected:
    Case* aCase;


    virtual void SetUp() {
    	aCase = new Case(VALID_COORDINATE_I, VALID_COORDINATE_J, VALID_COORDINATE_K);
    }

    virtual void TearDown() {
		delete aCase;
		aCase = NULL;
	}
  };


  TEST_F(CaseTest, initCase) {
	  ASSERT_EQ(MAXIMUM_NUMBER_OF_POSSIBILITIES, aCase->numberOfPossibilitiesRemaining());
  }

  TEST_F(CaseTest, initCaseWithValidCoordinates) {
	  ASSERT_EQ(VALID_COORDINATE_I, aCase->getI());
	  ASSERT_EQ(VALID_COORDINATE_J, aCase->getJ());
	  ASSERT_EQ(VALID_COORDINATE_K, aCase->getK());
	  ASSERT_EQ(MAXIMUM_NUMBER_OF_POSSIBILITIES, aCase->numberOfPossibilitiesRemaining());
  }

  TEST_F(CaseTest, initCaseWithInvalidCoordinates) {
	  delete aCase;
	  aCase = NULL;
	  aCase = new Case(INVALID_COORDINATE, INVALID_COORDINATE, INVALID_COORDINATE);

	  ASSERT_NE(VALID_COORDINATE_I, aCase->getI());
	  ASSERT_NE(VALID_COORDINATE_J, aCase->getJ());
	  ASSERT_NE(VALID_COORDINATE_K, aCase->getK());
	  ASSERT_EQ(MAXIMUM_NUMBER_OF_POSSIBILITIES, aCase->numberOfPossibilitiesRemaining());
  }

  TEST_F(CaseTest, canSetAFinalValueForTheCase) {
	  aCase->setValue(A_VALID_SUDOKUBE_CASE_VALUE);

	  ASSERT_TRUE(aCase->isSolved());
	  ASSERT_EQ(1, aCase->numberOfPossibilitiesRemaining());
	  ASSERT_EQ(A_VALID_SUDOKUBE_CASE_VALUE, aCase->getPossibilities().front());
  }

  TEST_F(CaseTest, cannotSetAWrongFinalValueForTheCase) {
	  aCase->setValue(A_INVALID_SUDOKUBE_CASE_VALUE);

	  ASSERT_FALSE(aCase->isSolved());
	  ASSERT_EQ(0, aCase->getValue());
  }

  TEST_F(CaseTest, canRemoveAPossibility) {
	  aCase->removePossibility(1);

	  ASSERT_EQ(7, aCase->numberOfPossibilitiesRemaining());
  }

  TEST_F(CaseTest, isSolvedWhenSevenPossibilitiesRemoved) {
	  aCase->removePossibility(1);
	  aCase->removePossibility(2);
	  aCase->removePossibility(3);
	  aCase->removePossibility(4);
	  aCase->removePossibility(5);
	  aCase->removePossibility(6);
	  aCase->removePossibility(7);

	  ASSERT_EQ(1, aCase->numberOfPossibilitiesRemaining());
	  ASSERT_TRUE(aCase->isSolved());
  }

}
