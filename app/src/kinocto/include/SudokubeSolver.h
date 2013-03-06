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
	void bruteForce(Sudokube&);

private:
	bool removePossibilityFromListOfCases(vector<Case*>,int);
	bool removePossibilitiesFromNakedPairsWithinAListOfCases(vector<Case*>,int,int);
	void solveWithoutBruteForce(Sudokube&);
};

#endif /* SUDOKUBESOLVER_H_ */
