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
	bool nakedPairs(Sudokube&);
	bool hiddenPairs(Sudokube&);
	bool hiddenTriples(Sudokube&);
	bool pointingPairs(Sudokube&);
	bool pointingTriples(Sudokube&);
	bool boxLineReductionPair(Sudokube&);
	bool boxLineReductionTriple(Sudokube&);
	bool xWing(Sudokube&);
	void bruteForce(Sudokube&);

private:
	Sudokube sudokube;
	void solveWithoutBruteForce(Sudokube&);

};

#endif /* SUDOKUBESOLVER_H_ */
