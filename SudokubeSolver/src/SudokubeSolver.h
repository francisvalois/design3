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

	void solve(Sudokube&);


	//STRATEGIES
	bool lastRemainingCellInARegion(Sudokube&);
	bool simpleConstraintPropagation(Sudokube&);

private:
	Sudokube sudokube;

};

#endif /* SUDOKUBESOLVER_H_ */
