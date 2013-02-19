/*
 * CaseTest.cpp
 *
 *  Created on: 2013-02-13
 *      Author: olivier
 */

#include "gtest/gtest.h"
#include "Case.h"

namespace {
  class CaseTest : public ::testing::Test {
  protected:
    Case* aCase;

    virtual void SetUp() {
    	aCase = new Case();
    }

    virtual void TearDown() {
		delete aCase;
		aCase = NULL;
	}
  };

  const int A_VALID_SUDOKUBE_VALUE = 8;
  const int A_INVALID_SUDOKUBE_VALUE = 9;


  TEST_F(CaseTest, initCase) {
	  ASSERT_EQ(8, aCase->numberOfPossibilitiesRemaining());
  }

  TEST_F(CaseTest, canSetAFinalValueForTheCase) {
	  aCase->setValue(A_VALID_SUDOKUBE_VALUE);

	  ASSERT_EQ(1, aCase->numberOfPossibilitiesRemaining());
	  ASSERT_EQ(A_VALID_SUDOKUBE_VALUE, aCase->getPossibilities().front());
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
