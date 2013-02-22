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
    Sudokube sudokube;
    SudokubeSolver solver;

    virtual void SetUp() {
    	//sudokube = new Sudokube();
    }

    virtual void TearDown() {
		//delete sudokube;
		//sudokube = NULL;
	}
  };


  TEST_F(SudokubeSolverTest, print) {



	  sudokube.print();
	  solver.solve(sudokube);
	  sudokube.print();
	  ASSERT_TRUE(1);
  }
}

//SUDOKUBE 3
//sudokube.setCaseValue(1,1,2,6);
//sudokube.setCaseValue(1,1,3,2);
//sudokube.setCaseValue(1,1,4,8);
//sudokube.setCaseValue(1,4,1,4);
//sudokube.setCaseValue(2,1,2,5);
//sudokube.setCaseValue(2,2,3,2);
//sudokube.setCaseValue(2,3,2,8);
//sudokube.setCaseValue(3,1,1,1);
//sudokube.setCaseValue(3,2,4,4);
//sudokube.setCaseValue(3,4,3,3);

//	  sudokube.setCaseValue(1,1,1,5);
//	  sudokube.setCaseValue(1,2,1,1);
//	  sudokube.setCaseValue(1,2,2,7);
//	  sudokube.setCaseValue(1,2,3,3);
//	  sudokube.setCaseValue(1,2,4,4);
//	  sudokube.setCaseValue(1,3,1,8);
//	  sudokube.setCaseValue(1,3,2,3);
//	  sudokube.setCaseValue(1,3,3,1);
//	  sudokube.setCaseValue(1,3,4,6);
//	  sudokube.setCaseValue(1,4,2,2);
//	  sudokube.setCaseValue(1,4,3,7);
//	  sudokube.setCaseValue(1,4,4,5);
//	  sudokube.setCaseValue(2,1,1,3);
//	  sudokube.setCaseValue(2,1,3,7);
//	  sudokube.setCaseValue(2,1,4,8);
//	  sudokube.setCaseValue(2,2,1,4);
//	  sudokube.setCaseValue(2,2,2,6);
//	  sudokube.setCaseValue(2,2,4,1);
//	  sudokube.setCaseValue(2,3,1,7);
//	  sudokube.setCaseValue(2,3,3,4);
//	  sudokube.setCaseValue(2,3,4,6);
//	  sudokube.setCaseValue(2,4,1,1);
//	  sudokube.setCaseValue(2,4,2,2);
//	  sudokube.setCaseValue(2,4,3,5);
//	  sudokube.setCaseValue(2,4,4,3);
//	  sudokube.setCaseValue(3,1,2,3);
//	  sudokube.setCaseValue(3,1,3,2);
//	  sudokube.setCaseValue(3,1,4,7);
//	  sudokube.setCaseValue(3,2,1,6);
//	  sudokube.setCaseValue(3,2,2,8);
//	  sudokube.setCaseValue(3,2,3,5);
//	  sudokube.setCaseValue(3,3,1,4);
//	  sudokube.setCaseValue(3,3,2,5);
//	  sudokube.setCaseValue(3,3,3,1);
//	  sudokube.setCaseValue(3,3,4,8);
//	  sudokube.setCaseValue(3,4,1,2);
//	  sudokube.setCaseValue(3,4,2,7);
//	  sudokube.setCaseValue(3,4,4,6);



//SUDOKUBE2

//	  sudokube.setCaseValue(1,1,1,8);
//	  sudokube.setCaseValue(1,2,2,1);
//	  sudokube.setCaseValue(1,3,2,4);
//	  sudokube.setCaseValue(1,3,3,7);
//	  sudokube.setCaseValue(1,4,4,3);
//	  sudokube.setCaseValue(2,2,2,6);
//	  sudokube.setCaseValue(2,3,2,4);
//	  sudokube.setCaseValue(2,1,4,5);
//	  sudokube.setCaseValue(3,4,1,2);
//	  sudokube.setCaseValue(3,1,1,8);
//	  sudokube.setCaseValue(3,2,2,1);



//SUDOKUBE1

//	  sudokube.setCaseValue(1,1,3,7);
//	  sudokube.setCaseValue(1,2,1,6);
//	  sudokube.setCaseValue(1,2,4,2);
//	  sudokube.setCaseValue(1,3,2,5);
//	  sudokube.setCaseValue(1,4,1,7);
//	  sudokube.setCaseValue(2,1,2,4);
//	  sudokube.setCaseValue(2,1,4,6);
//	  sudokube.setCaseValue(2,2,3,7);
//	  sudokube.setCaseValue(2,3,1,2);
//	  sudokube.setCaseValue(2,4,2,1);
//	  sudokube.setCaseValue(3,1,1,3);
//	  sudokube.setCaseValue(3,2,3,1);
//	  sudokube.setCaseValue(3,4,1,5);
//	  sudokube.setCaseValue(3,4,4,8);
