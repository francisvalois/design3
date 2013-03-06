#include <gtest/gtest.h>

#include "Sudokube.h"
#include "SudokubeSolver.h"

namespace {
  class SudokubeSolverTest : public ::testing::Test {
  protected:
    Sudokube sudokube;
    SudokubeSolver solver;
  };


  TEST_F(SudokubeSolverTest, sudokubeIT1) {

	  sudokube.setCaseValue(1,1,2,6);
	  sudokube.setCaseValue(1,1,3,2);
	  sudokube.setCaseValue(1,1,4,8);
	  sudokube.setCaseValue(1,4,1,4);
	  sudokube.setCaseValue(2,1,2,5);
	  sudokube.setCaseValue(2,2,3,2);
	  sudokube.setCaseValue(2,3,2,8);
	  sudokube.setCaseValue(3,1,1,1);
	  sudokube.setCaseValue(3,2,4,4);
	  sudokube.setCaseValue(3,4,3,3);

	  solver.solve(sudokube);

// Sudokube should look like
//	     7 4 8 6
//	     2 5 1 3
//	     3 8 5 7
//	     1 6 4 2
//	5 7 2 4   8 1 6 3
//	6 1 3 8   7 2 4 5
//	4 3 7 1   5 6 8 2
//	8 2 6 5   3 4 7 1

	  ASSERT_EQ(5, sudokube.getCaseValue(1,1,1));
	  ASSERT_EQ(6, sudokube.getCaseValue(1,1,2));
	  ASSERT_EQ(2, sudokube.getCaseValue(1,1,3));
	  ASSERT_EQ(8, sudokube.getCaseValue(1,1,4));
	  ASSERT_EQ(1, sudokube.getCaseValue(1,2,1));
	  ASSERT_EQ(7, sudokube.getCaseValue(1,2,2));
	  ASSERT_EQ(3, sudokube.getCaseValue(1,2,3));
	  ASSERT_EQ(4, sudokube.getCaseValue(1,2,4));
	  ASSERT_EQ(8, sudokube.getCaseValue(1,3,1));
	  ASSERT_EQ(3, sudokube.getCaseValue(1,3,2));
	  ASSERT_EQ(1, sudokube.getCaseValue(1,3,3));
	  ASSERT_EQ(6, sudokube.getCaseValue(1,3,4));
	  ASSERT_EQ(4, sudokube.getCaseValue(1,4,1));
	  ASSERT_EQ(2, sudokube.getCaseValue(1,4,2));
	  ASSERT_EQ(7, sudokube.getCaseValue(1,4,3));
	  ASSERT_EQ(5, sudokube.getCaseValue(1,4,4));
	  ASSERT_EQ(3, sudokube.getCaseValue(2,1,1));
	  ASSERT_EQ(5, sudokube.getCaseValue(2,1,2));
	  ASSERT_EQ(7, sudokube.getCaseValue(2,1,3));
	  ASSERT_EQ(8, sudokube.getCaseValue(2,1,4));
	  ASSERT_EQ(4, sudokube.getCaseValue(2,2,1));
	  ASSERT_EQ(6, sudokube.getCaseValue(2,2,2));
	  ASSERT_EQ(2, sudokube.getCaseValue(2,2,3));
	  ASSERT_EQ(1, sudokube.getCaseValue(2,2,4));
	  ASSERT_EQ(7, sudokube.getCaseValue(2,3,1));
	  ASSERT_EQ(8, sudokube.getCaseValue(2,3,2));
	  ASSERT_EQ(4, sudokube.getCaseValue(2,3,3));
	  ASSERT_EQ(6, sudokube.getCaseValue(2,3,4));
	  ASSERT_EQ(1, sudokube.getCaseValue(2,4,1));
	  ASSERT_EQ(2, sudokube.getCaseValue(2,4,2));
	  ASSERT_EQ(5, sudokube.getCaseValue(2,4,3));
	  ASSERT_EQ(3, sudokube.getCaseValue(2,4,4));
	  ASSERT_EQ(1, sudokube.getCaseValue(3,1,1));
	  ASSERT_EQ(3, sudokube.getCaseValue(3,1,2));
	  ASSERT_EQ(2, sudokube.getCaseValue(3,1,3));
	  ASSERT_EQ(7, sudokube.getCaseValue(3,1,4));
	  ASSERT_EQ(6, sudokube.getCaseValue(3,2,1));
	  ASSERT_EQ(8, sudokube.getCaseValue(3,2,2));
	  ASSERT_EQ(5, sudokube.getCaseValue(3,2,3));
	  ASSERT_EQ(4, sudokube.getCaseValue(3,2,4));
	  ASSERT_EQ(4, sudokube.getCaseValue(3,3,1));
	  ASSERT_EQ(5, sudokube.getCaseValue(3,3,2));
	  ASSERT_EQ(1, sudokube.getCaseValue(3,3,3));
	  ASSERT_EQ(8, sudokube.getCaseValue(3,3,4));
	  ASSERT_EQ(2, sudokube.getCaseValue(3,4,1));
	  ASSERT_EQ(7, sudokube.getCaseValue(3,4,2));
	  ASSERT_EQ(3, sudokube.getCaseValue(3,4,3));
	  ASSERT_EQ(6, sudokube.getCaseValue(3,4,4));
  }

  TEST_F(SudokubeSolverTest, sudokubeIT2) {

  	  sudokube.setCaseValue(1,1,1,6);
  	  sudokube.setCaseValue(1,1,2,2);
  	  sudokube.setCaseValue(1,2,3,3);
  	  sudokube.setCaseValue(2,1,2,7);
  	  sudokube.setCaseValue(2,2,1,1);
  	  sudokube.setCaseValue(2,3,2,4);
  	  sudokube.setCaseValue(2,3,3,3);
  	  sudokube.setCaseValue(3,2,4,7);
  	  sudokube.setCaseValue(3,3,3,8);

  	  solver.solve(sudokube);

// Sudokube should look like
//      4 7 5 3
//      6 2 8 1
//      3 5 7 4
//      1 8 6 2
// 5 1 3 8   4 6 7 2
// 2 6 4 7   5 8 3 1
// 8 3 1 5   7 2 4 6
// 7 4 2 6   3 1 5 8

  	  ASSERT_EQ(6, sudokube.getCaseValue(1,1,1));
  	  ASSERT_EQ(2, sudokube.getCaseValue(1,1,2));
  	  ASSERT_EQ(4, sudokube.getCaseValue(1,1,3));
  	  ASSERT_EQ(7, sudokube.getCaseValue(1,1,4));
  	  ASSERT_EQ(5, sudokube.getCaseValue(1,2,1));
  	  ASSERT_EQ(1, sudokube.getCaseValue(1,2,2));
  	  ASSERT_EQ(3, sudokube.getCaseValue(1,2,3));
  	  ASSERT_EQ(8, sudokube.getCaseValue(1,2,4));
  	  ASSERT_EQ(7, sudokube.getCaseValue(1,3,1));
  	  ASSERT_EQ(4, sudokube.getCaseValue(1,3,2));
  	  ASSERT_EQ(6, sudokube.getCaseValue(1,3,3));
  	  ASSERT_EQ(2, sudokube.getCaseValue(1,3,4));
  	  ASSERT_EQ(8, sudokube.getCaseValue(1,4,1));
  	  ASSERT_EQ(3, sudokube.getCaseValue(1,4,2));
  	  ASSERT_EQ(1, sudokube.getCaseValue(1,4,3));
  	  ASSERT_EQ(5, sudokube.getCaseValue(1,4,4));
  	  ASSERT_EQ(3, sudokube.getCaseValue(2,1,1));
  	  ASSERT_EQ(7, sudokube.getCaseValue(2,1,2));
  	  ASSERT_EQ(5, sudokube.getCaseValue(2,1,3));
  	  ASSERT_EQ(4, sudokube.getCaseValue(2,1,4));
  	  ASSERT_EQ(1, sudokube.getCaseValue(2,2,1));
  	  ASSERT_EQ(2, sudokube.getCaseValue(2,2,2));
  	  ASSERT_EQ(8, sudokube.getCaseValue(2,2,3));
  	  ASSERT_EQ(6, sudokube.getCaseValue(2,2,4));
  	  ASSERT_EQ(5, sudokube.getCaseValue(2,3,1));
  	  ASSERT_EQ(4, sudokube.getCaseValue(2,3,2));
  	  ASSERT_EQ(3, sudokube.getCaseValue(2,3,3));
  	  ASSERT_EQ(7, sudokube.getCaseValue(2,3,4));
  	  ASSERT_EQ(8, sudokube.getCaseValue(2,4,1));
  	  ASSERT_EQ(6, sudokube.getCaseValue(2,4,2));
  	  ASSERT_EQ(1, sudokube.getCaseValue(2,4,3));
  	  ASSERT_EQ(2, sudokube.getCaseValue(2,4,4));
  	  ASSERT_EQ(1, sudokube.getCaseValue(3,1,1));
  	  ASSERT_EQ(3, sudokube.getCaseValue(3,1,2));
  	  ASSERT_EQ(6, sudokube.getCaseValue(3,1,3));
  	  ASSERT_EQ(4, sudokube.getCaseValue(3,1,4));
  	  ASSERT_EQ(8, sudokube.getCaseValue(3,2,1));
  	  ASSERT_EQ(5, sudokube.getCaseValue(3,2,2));
  	  ASSERT_EQ(2, sudokube.getCaseValue(3,2,3));
  	  ASSERT_EQ(7, sudokube.getCaseValue(3,2,4));
  	  ASSERT_EQ(6, sudokube.getCaseValue(3,3,1));
  	  ASSERT_EQ(7, sudokube.getCaseValue(3,3,2));
  	  ASSERT_EQ(8, sudokube.getCaseValue(3,3,3));
  	  ASSERT_EQ(5, sudokube.getCaseValue(3,3,4));
  	  ASSERT_EQ(2, sudokube.getCaseValue(3,4,1));
  	  ASSERT_EQ(4, sudokube.getCaseValue(3,4,2));
  	  ASSERT_EQ(1, sudokube.getCaseValue(3,4,3));
  	  ASSERT_EQ(3, sudokube.getCaseValue(3,4,4));
    }

  TEST_F(SudokubeSolverTest, sudokubeIT3) {

	  sudokube.setCaseValue(1,1,3,7);
	  sudokube.setCaseValue(1,2,1,6);
	  sudokube.setCaseValue(1,2,4,2);
	  sudokube.setCaseValue(1,3,2,5);
	  sudokube.setCaseValue(1,4,1,7);
	  sudokube.setCaseValue(2,1,2,4);
	  sudokube.setCaseValue(2,1,4,6);
	  sudokube.setCaseValue(2,2,3,7);
	  sudokube.setCaseValue(2,3,1,2);
	  sudokube.setCaseValue(2,4,2,1);
	  sudokube.setCaseValue(3,1,1,3);
	  sudokube.setCaseValue(3,2,3,1);
	  sudokube.setCaseValue(3,4,1,5);
	  sudokube.setCaseValue(3,4,4,8);

  	  solver.solve(sudokube);

  // Sudokube should look like
//      7 5 2 8
//      4 1 6 3
//      8 6 1 4
//      3 2 7 5
// 1 3 8 7   6 2 5 4
// 6 4 5 2   1 7 8 3
// 2 8 3 6   4 5 7 1
// 5 7 4 1   8 3 2 6

  	  ASSERT_EQ(1, sudokube.getCaseValue(1,1,1));
  	  ASSERT_EQ(4, sudokube.getCaseValue(1,1,2));
  	  ASSERT_EQ(7, sudokube.getCaseValue(1,1,3));
  	  ASSERT_EQ(5, sudokube.getCaseValue(1,1,4));
  	  ASSERT_EQ(6, sudokube.getCaseValue(1,2,1));
  	  ASSERT_EQ(3, sudokube.getCaseValue(1,2,2));
  	  ASSERT_EQ(8, sudokube.getCaseValue(1,2,3));
  	  ASSERT_EQ(2, sudokube.getCaseValue(1,2,4));
  	  ASSERT_EQ(2, sudokube.getCaseValue(1,3,1));
  	  ASSERT_EQ(5, sudokube.getCaseValue(1,3,2));
  	  ASSERT_EQ(4, sudokube.getCaseValue(1,3,3));
  	  ASSERT_EQ(6, sudokube.getCaseValue(1,3,4));
  	  ASSERT_EQ(7, sudokube.getCaseValue(1,4,1));
  	  ASSERT_EQ(8, sudokube.getCaseValue(1,4,2));
  	  ASSERT_EQ(3, sudokube.getCaseValue(1,4,3));
  	  ASSERT_EQ(1, sudokube.getCaseValue(1,4,4));
  	  ASSERT_EQ(8, sudokube.getCaseValue(2,1,1));
  	  ASSERT_EQ(4, sudokube.getCaseValue(2,1,2));
  	  ASSERT_EQ(1, sudokube.getCaseValue(2,1,3));
  	  ASSERT_EQ(6, sudokube.getCaseValue(2,1,4));
  	  ASSERT_EQ(3, sudokube.getCaseValue(2,2,1));
  	  ASSERT_EQ(5, sudokube.getCaseValue(2,2,2));
  	  ASSERT_EQ(7, sudokube.getCaseValue(2,2,3));
  	  ASSERT_EQ(2, sudokube.getCaseValue(2,2,4));
  	  ASSERT_EQ(2, sudokube.getCaseValue(2,3,1));
  	  ASSERT_EQ(7, sudokube.getCaseValue(2,3,2));
  	  ASSERT_EQ(8, sudokube.getCaseValue(2,3,3));
  	  ASSERT_EQ(5, sudokube.getCaseValue(2,3,4));
  	  ASSERT_EQ(6, sudokube.getCaseValue(2,4,1));
  	  ASSERT_EQ(1, sudokube.getCaseValue(2,4,2));
  	  ASSERT_EQ(3, sudokube.getCaseValue(2,4,3));
  	  ASSERT_EQ(4, sudokube.getCaseValue(2,4,4));
  	  ASSERT_EQ(3, sudokube.getCaseValue(3,1,1));
  	  ASSERT_EQ(8, sudokube.getCaseValue(3,1,2));
  	  ASSERT_EQ(4, sudokube.getCaseValue(3,1,3));
  	  ASSERT_EQ(7, sudokube.getCaseValue(3,1,4));
  	  ASSERT_EQ(2, sudokube.getCaseValue(3,2,1));
  	  ASSERT_EQ(6, sudokube.getCaseValue(3,2,2));
  	  ASSERT_EQ(1, sudokube.getCaseValue(3,2,3));
  	  ASSERT_EQ(5, sudokube.getCaseValue(3,2,4));
  	  ASSERT_EQ(7, sudokube.getCaseValue(3,3,1));
  	  ASSERT_EQ(1, sudokube.getCaseValue(3,3,2));
  	  ASSERT_EQ(6, sudokube.getCaseValue(3,3,3));
  	  ASSERT_EQ(2, sudokube.getCaseValue(3,3,4));
  	  ASSERT_EQ(5, sudokube.getCaseValue(3,4,1));
  	  ASSERT_EQ(4, sudokube.getCaseValue(3,4,2));
  	  ASSERT_EQ(3, sudokube.getCaseValue(3,4,3));
  	  ASSERT_EQ(8, sudokube.getCaseValue(3,4,4));
    }

  TEST_F(SudokubeSolverTest, sudokubeIT4) {

	  sudokube.setCaseValue(1,1,1,8);
	  sudokube.setCaseValue(1,2,2,1);
	  sudokube.setCaseValue(1,3,2,4);
	  sudokube.setCaseValue(1,3,3,7);
	  sudokube.setCaseValue(1,4,4,3);
	  sudokube.setCaseValue(2,2,2,6);
	  sudokube.setCaseValue(2,3,2,4);
	  sudokube.setCaseValue(2,1,4,5);
	  sudokube.setCaseValue(3,4,1,2);
	  sudokube.setCaseValue(3,1,1,8);
	  sudokube.setCaseValue(3,2,2,1);

  	  solver.solve(sudokube);

  	  ASSERT_EQ(8, sudokube.getCaseValue(1,1,1));
  	  ASSERT_EQ(7, sudokube.getCaseValue(1,1,2));
  	  ASSERT_EQ(6, sudokube.getCaseValue(1,1,3));
  	  ASSERT_EQ(4, sudokube.getCaseValue(1,1,4));
  	  ASSERT_EQ(3, sudokube.getCaseValue(1,2,1));
  	  ASSERT_EQ(1, sudokube.getCaseValue(1,2,2));
  	  ASSERT_EQ(5, sudokube.getCaseValue(1,2,3));
  	  ASSERT_EQ(2, sudokube.getCaseValue(1,2,4));
  	  ASSERT_EQ(5, sudokube.getCaseValue(1,3,1));
  	  ASSERT_EQ(4, sudokube.getCaseValue(1,3,2));
  	  ASSERT_EQ(7, sudokube.getCaseValue(1,3,3));
  	  ASSERT_EQ(1, sudokube.getCaseValue(1,3,4));
  	  ASSERT_EQ(6, sudokube.getCaseValue(1,4,1));
  	  ASSERT_EQ(2, sudokube.getCaseValue(1,4,2));
  	  ASSERT_EQ(8, sudokube.getCaseValue(1,4,3));
  	  ASSERT_EQ(3, sudokube.getCaseValue(1,4,4));
  // Sudokube should look like
//      7 2 5 4
//      6 3 8 1
//      5 1 3 7
//      8 4 6 2
// 3 8 2 6   5 4 7 1
// 1 7 4 5   3 8 2 6
// 2 5 1 3   7 6 4 8
// 4 6 7 8   1 2 5 3
  	  ASSERT_EQ(1, sudokube.getCaseValue(2,1,1));
  	  ASSERT_EQ(7, sudokube.getCaseValue(2,1,2));
  	  ASSERT_EQ(3, sudokube.getCaseValue(2,1,3));
  	  ASSERT_EQ(5, sudokube.getCaseValue(2,1,4));
  	  ASSERT_EQ(2, sudokube.getCaseValue(2,2,1));
  	  ASSERT_EQ(6, sudokube.getCaseValue(2,2,2));
  	  ASSERT_EQ(8, sudokube.getCaseValue(2,2,3));
  	  ASSERT_EQ(4, sudokube.getCaseValue(2,2,4));
  	  ASSERT_EQ(5, sudokube.getCaseValue(2,3,1));
  	  ASSERT_EQ(4, sudokube.getCaseValue(2,3,2));
  	  ASSERT_EQ(2, sudokube.getCaseValue(2,3,3));
  	  ASSERT_EQ(7, sudokube.getCaseValue(2,3,4));
  	  ASSERT_EQ(3, sudokube.getCaseValue(2,4,1));
  	  ASSERT_EQ(8, sudokube.getCaseValue(2,4,2));
  	  ASSERT_EQ(6, sudokube.getCaseValue(2,4,3));
  	  ASSERT_EQ(1, sudokube.getCaseValue(2,4,4));
  	  ASSERT_EQ(8, sudokube.getCaseValue(3,1,1));
  	  ASSERT_EQ(5, sudokube.getCaseValue(3,1,2));
  	  ASSERT_EQ(6, sudokube.getCaseValue(3,1,3));
  	  ASSERT_EQ(7, sudokube.getCaseValue(3,1,4));
  	  ASSERT_EQ(4, sudokube.getCaseValue(3,2,1));
  	  ASSERT_EQ(1, sudokube.getCaseValue(3,2,2));
  	  ASSERT_EQ(3, sudokube.getCaseValue(3,2,3));
  	  ASSERT_EQ(2, sudokube.getCaseValue(3,2,4));
  	  ASSERT_EQ(6, sudokube.getCaseValue(3,3,1));
  	  ASSERT_EQ(3, sudokube.getCaseValue(3,3,2));
  	  ASSERT_EQ(8, sudokube.getCaseValue(3,3,3));
  	  ASSERT_EQ(5, sudokube.getCaseValue(3,3,4));
  	  ASSERT_EQ(2, sudokube.getCaseValue(3,4,1));
  	  ASSERT_EQ(7, sudokube.getCaseValue(3,4,2));
  	  ASSERT_EQ(1, sudokube.getCaseValue(3,4,3));
  	  ASSERT_EQ(4, sudokube.getCaseValue(3,4,4));
    }
}
