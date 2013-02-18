/*
 * SudokubeTest.cpp
 *
 *  Created on: 2013-02-13
 *      Author: olivier
 */

#include "gtest/gtest.h"
#include "Sudokube.h"
#include "SudokubeSolver.h"

namespace {
  class SudokubeSolverTest : public ::testing::Test {
  protected:
    Sudokube* sudokube;
    SudokubeSolver solver;

    virtual void SetUp() {
    	sudokube = new Sudokube();
    }

    virtual void TearDown() {
		delete sudokube;
		sudokube = NULL;
	}
  };


  TEST_F(SudokubeSolverTest, print) {
	  sudokube->print();
	  cout << endl;
	  sudokube->setCase(1,1,1,8);
	  sudokube->setCase(1,1,2,3);
	  solver.solve(*sudokube);
	  sudokube->print();
	  ASSERT_TRUE(1);
  }
}
