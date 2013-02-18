/*
 * SudokubeSolver.h
 *
 *  Created on: 2013-02-13
 *      Author: olivier
 */

#ifndef SUDOKUBESOLVER_H_
#define SUDOKUBESOLVER_H_

#include "Sudokube.h"

class SudokubeSolver {
public:
	SudokubeSolver();
	virtual ~SudokubeSolver();

	Sudokube solve(Sudokube);


	//STRATEGIES
	bool lastRemainingCellInABox(Sudokube);

private:
	Sudokube sudokube;

};

#endif /* SUDOKUBESOLVER_H_ */
