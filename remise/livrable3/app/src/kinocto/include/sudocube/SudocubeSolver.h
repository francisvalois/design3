#ifndef SUDOKUBESOLVER_H_
#define SUDOKUBESOLVER_H_

#include <vector>

#include "sudocube/Sudocube.h"

class SudocubeSolver {
public:
    SudocubeSolver();
    virtual ~SudocubeSolver();

    void solve(Sudocube&);

private:
    bool removePossibilityFromListOfCases(std::vector<Case*>, int);
    bool removePossibilitiesFromNakedPairsWithinAListOfCases(std::vector<Case*>, int, int);
    void solveWithoutBruteForce(Sudocube&);

    //STRATEGIES
    bool lastRemainingCellInARegion(Sudocube&);
    bool simpleConstraintPropagation(Sudocube&);
    bool nakedPairs(Sudocube&);
    bool hiddenPairs(Sudocube&);
    bool hiddenTriples(Sudocube&);
    bool pointingPairs(Sudocube&);
    bool pointingTriples(Sudocube&);
    bool boxLineReductionPair(Sudocube&);
    bool boxLineReductionTriple(Sudocube&);
    void bruteForce(Sudocube&);
};

#endif /* SUDOKUBESOLVER_H_ */
