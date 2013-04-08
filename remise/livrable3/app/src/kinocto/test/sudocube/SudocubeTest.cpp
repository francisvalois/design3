#include <gtest/gtest.h>
#include "sudocube/Sudocube.h"

namespace {
  class SudocubeTest : public ::testing::Test {
  protected:
    Sudocube* sudocube;

    virtual void SetUp() {
    	sudocube = new Sudocube();
    }

    virtual void TearDown() {
		delete sudocube;
		sudocube = NULL;
	}
  };


  TEST_F(SudocubeTest, initSudocube) {
	  ASSERT_FALSE(sudocube->isSolved());
  }

  TEST_F(SudocubeTest, canSolveSudocubeBySettingCase) {
		for(int i = 1; i <= 3; i++) {
			for(int j = 1; j <= 4; j++) {
				for(int k = 1; k <= 4; k++) {
					sudocube->setCaseValue(i,j,k,8);
				}
			}
		}
	  ASSERT_TRUE(sudocube->isSolved());
  }

  TEST_F(SudocubeTest, canSolveSudocubeByRemovingPossibility) {
		for(int i = 1; i <= 3; i++) {
			for(int j = 1; j <= 4; j++) {
				for(int k = 1; k <= 4; k++) {
					if(i == 3 && j == 4 && k == 4) {
						//do nothing
					}else {
						sudocube->setCaseValue(i,j,k,8);
					}
				}
			}
		}

		ASSERT_FALSE(sudocube->isSolved());

		sudocube->removePossibility(3,4,4,1);
		sudocube->removePossibility(3,4,4,2);
		sudocube->removePossibility(3,4,4,3);
		sudocube->removePossibility(3,4,4,4);
		sudocube->removePossibility(3,4,4,5);
		sudocube->removePossibility(3,4,4,6);
		sudocube->removePossibility(3,4,4,7);

		ASSERT_TRUE(sudocube->isSolved());
  }

  TEST_F(SudocubeTest, sodokuIsntSolvedWhenUsingWrongIndexes) {
		for(int i = 1; i <= 3; i++) {
			for(int j = 1; j <= 4; j++) {
				for(int k = 1; k <= 4; k++) {
					if(i == 3 && j == 4 && k == 4) {
						//do nothing
					}else {
						sudocube->setCaseValue(i,j,k,8);
					}
				}
			}
		}

		ASSERT_FALSE(sudocube->isSolved());

		sudocube->removePossibility(3,4,4,1);
		sudocube->removePossibility(3,4,4,2);
		sudocube->removePossibility(3,4,4,3);
		sudocube->removePossibility(3,4,4,4);
		sudocube->removePossibility(3,4,4,5);
		sudocube->removePossibility(3,4,4,6);
		sudocube->removePossibility(3,4,5,7);

		ASSERT_FALSE(sudocube->isSolved());
  }

  TEST_F(SudocubeTest, emptySudocubeAreEquals) {
      Sudocube sudocube1;
      Sudocube sudocube2;

      ASSERT_TRUE(sudocube1.equals(sudocube2));
  }

  TEST_F(SudocubeTest, aSudocubeIsEmptyIfNoValueHasBeenSet) {
      ASSERT_TRUE(sudocube->isEmpty());
  }

// The following three tests will not be tested with GTest. Since the Sudocube structure is
// static, the values can be verified easily and don't need to be tested here.

//  TEST_F(SudocubeTest, canGetTheSameColumnOfACase) {
//  }
//
//  TEST_F(SudocubeTest, canGetTheSameLineOfACase) {
//  }
//
//  TEST_F(SudocubeTest, canGetTheSameRegionOfACase) {
//  }

}
