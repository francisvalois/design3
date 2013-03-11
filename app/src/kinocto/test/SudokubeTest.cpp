#include <gtest/gtest.h>
#include "Sudokube.h"

namespace {
  class SudokubeTest : public ::testing::Test {
  protected:
    Sudokube* sudokube;

    virtual void SetUp() {
    	sudokube = new Sudokube();
    }

    virtual void TearDown() {
		delete sudokube;
		sudokube = NULL;
	}
  };


  TEST_F(SudokubeTest, initSudokube) {
	  ASSERT_FALSE(sudokube->isSolved());
  }

  TEST_F(SudokubeTest, canSolveSudokubeBySettingCase) {
		for(int i = 1; i <= 3; i++) {
			for(int j = 1; j <= 4; j++) {
				for(int k = 1; k <= 4; k++) {
					sudokube->setCaseValue(i,j,k,8);
				}
			}
		}
	  ASSERT_TRUE(sudokube->isSolved());
  }

  TEST_F(SudokubeTest, canSolveSudokubeByRemovingPossibility) {
		for(int i = 1; i <= 3; i++) {
			for(int j = 1; j <= 4; j++) {
				for(int k = 1; k <= 4; k++) {
					if(i == 3 && j == 4 && k == 4) {
						//do nothing
					}else {
						sudokube->setCaseValue(i,j,k,8);
					}
				}
			}
		}

		ASSERT_FALSE(sudokube->isSolved());

		sudokube->removePossibility(3,4,4,1);
		sudokube->removePossibility(3,4,4,2);
		sudokube->removePossibility(3,4,4,3);
		sudokube->removePossibility(3,4,4,4);
		sudokube->removePossibility(3,4,4,5);
		sudokube->removePossibility(3,4,4,6);
		sudokube->removePossibility(3,4,4,7);

		ASSERT_TRUE(sudokube->isSolved());
  }

  TEST_F(SudokubeTest, sodokuIsntSolvedWhenUsingWrongIndexes) {
		for(int i = 1; i <= 3; i++) {
			for(int j = 1; j <= 4; j++) {
				for(int k = 1; k <= 4; k++) {
					if(i == 3 && j == 4 && k == 4) {
						//do nothing
					}else {
						sudokube->setCaseValue(i,j,k,8);
					}
				}
			}
		}

		ASSERT_FALSE(sudokube->isSolved());

		sudokube->removePossibility(3,4,4,1);
		sudokube->removePossibility(3,4,4,2);
		sudokube->removePossibility(3,4,4,3);
		sudokube->removePossibility(3,4,4,4);
		sudokube->removePossibility(3,4,4,5);
		sudokube->removePossibility(3,4,4,6);
		sudokube->removePossibility(3,4,5,7);

		ASSERT_FALSE(sudokube->isSolved());
  }

  TEST_F(SudokubeTest, emptySudokubeAreEquals) {
      Sudokube sudokube1;
      Sudokube sudokube2;

      ASSERT_TRUE(sudokube1.equals(sudokube2));
  }

  TEST_F(SudokubeTest, aSudokubeIsEmptyIfNoValueHasBeenSet) {
      ASSERT_TRUE(sudokube->isEmpty());
  }

// The following three tests will not be tested with GTest. Since the sudokube structure is
// static, the values can be verified easily and don't need to be tested here.

//  TEST_F(SudokubeTest, canGetTheSameColumnOfACase) {
//  }
//
//  TEST_F(SudokubeTest, canGetTheSameLineOfACase) {
//  }
//
//  TEST_F(SudokubeTest, canGetTheSameRegionOfACase) {
//  }

}
