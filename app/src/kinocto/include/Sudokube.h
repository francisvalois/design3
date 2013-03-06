#ifndef SUDOKUBE_H_
#define SUDOKUBE_H_

#include "Case.h"
#include <vector>

class Sudokube {
public:
	Sudokube();
	virtual ~Sudokube();

	bool isSolved();
	bool caseSolved(int,int,int);
	int remainingCellsToBeSolved();

	void setCaseValue(int,int,int,int);
	int getCaseValue(int,int,int);
	void setRedCase(int,int,int);
	int* getRedCase();
	int getRedCaseValue();

	void removePossibility(int,int,int,int);
	vector<int> getPossibilities(int,int,int);
	void setPossibilities(int,int,int, vector<int>);

	void print();

	vector<vector<Case*> > getListOfAllCaseLinesAndRegions();
	vector<vector<Case*> > getListOfAllCaseLines();
	vector<Case*> getSameLineOfCase(int,int,int);
	vector<Case*> getSameColumnOfCase(int,int,int);
	vector<Case*> getSameRegionOfCase(int,int,int);

private:
	Case* container[3][4][4];
	int redCase[3];

	bool indexesOk(int,int,int);


};

#endif /* SUDOKUBE_H_ */
