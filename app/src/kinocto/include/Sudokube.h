#ifndef SUDOKUBE_H_
#define SUDOKUBE_H_

#include "Case.h"
#include <vector>

class Sudokube {
public:
    Sudokube();
    virtual ~Sudokube();

    bool equals(Sudokube &);

    bool isSolved();
    bool caseSolved(int, int, int);
    int remainingCellsToBeSolved();

    void setCaseValue(int, int, int, int);
    int getCaseValue(int, int, int);
    void setRedCase(int, int, int);
    int* getRedCase();
    int getRedCaseValue();

    void removePossibility(int, int, int, int);
    std::vector<int> getPossibilities(int, int, int);
    void setPossibilities(int, int, int, std::vector<int>);

    void print();

    std::vector<std::vector<Case*> > getListOfAllCaseLinesAndRegions();
    std::vector<std::vector<Case*> > getListOfAllCaseLines();
    std::vector<Case*> getSameLineOfCase(int, int, int);
    std::vector<Case*> getSameColumnOfCase(int, int, int);
    std::vector<Case*> getSameRegionOfCase(int, int, int);

private:
    Case* container[3][4][4];
    int redCase[3];

    bool indexesOk(int, int, int);

};

#endif /* SUDOKUBE_H_ */
