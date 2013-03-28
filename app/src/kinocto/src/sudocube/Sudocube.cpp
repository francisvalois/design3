#include "sudocube/Sudocube.h"

const int CUBE_FACES = 3;
const int CUBE_FACE_HEIGHT = 4;
const int CUBE_FACE_LENGTH = 4;

using namespace std;

Sudocube::Sudocube() {
    for (int i = 0; i < CUBE_FACES; i++) {
        for (int j = 0; j < CUBE_FACE_HEIGHT; j++) {
            for (int k = 0; k < CUBE_FACE_LENGTH; k++) {
                container[i][j][k] = new Case(i + 1, j + 1, k + 1);
            }
        }
    }

    for (int i = 0; i < 3; i++) {
        redCase[i] = 0;
    }
}

Sudocube::~Sudocube() {
    Case* aCase;
    for (int i = 0; i < CUBE_FACES; i++) {
        for (int j = 0; j < CUBE_FACE_HEIGHT; j++) {
            for (int k = 0; k < CUBE_FACE_LENGTH; k++) {
                aCase = container[i][j][k];
                delete aCase;
                aCase = NULL;
            }
        }
    }
}

bool Sudocube::equals(Sudocube & s) {
    for (int i = 0; i < CUBE_FACES; i++) {
        for (int j = 0; j < CUBE_FACE_HEIGHT; j++) {
            for (int k = 0; k < CUBE_FACE_LENGTH; k++) {
                if (container[i][j][k]->getValue() != s.getCaseValue(i + 1, j + 1, k + 1)) {
                    return false;
                }
            }
        }
    }
    if (redCase[0] != s.getRedCase()[0] || redCase[1] != s.getRedCase()[1] || redCase[2] != s.getRedCase()[2]) {
        return false;
    }
    return true;
}

bool Sudocube::isSolved() {
    for (int i = 0; i < CUBE_FACES; i++) {
        for (int j = 0; j < CUBE_FACE_HEIGHT; j++) {
            for (int k = 0; k < CUBE_FACE_LENGTH; k++) {
                if (!(container[i][j][k]->isSolved())) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool Sudocube::isEmpty() {
    Sudocube Sudocube;

    return this->equals(Sudocube);
}

bool Sudocube::caseSolved(int i, int j, int k) {
    if (indexesOk(i, j, k)) {
        return container[i - 1][j - 1][k - 1]->isSolved();
    }
    return false;
}

int Sudocube::remainingCellsToBeSolved() {
    int remainingCells = 0;
    for (int i = 0; i < CUBE_FACES; i++) {
        for (int j = 0; j < CUBE_FACE_HEIGHT; j++) {
            for (int k = 0; k < CUBE_FACE_LENGTH; k++) {
                if (!(container[i][j][k]->isSolved())) {
                    remainingCells++;
                }
            }
        }
    }
    return remainingCells;
}

void Sudocube::setCaseValue(int i, int j, int k, int value) {
    if (indexesOk(i, j, k)) {
        container[i - 1][j - 1][k - 1]->setValue(value);
    }
}

int Sudocube::getCaseValue(int i, int j, int k) {
    if (indexesOk(i, j, k)) {
        return container[i - 1][j - 1][k - 1]->getValue();
    }
    return 0;
}

void Sudocube::setRedCase(int i, int j, int k) {
    if (indexesOk(i, j, k)) {
        redCase[0] = i;
        redCase[1] = j;
        redCase[2] = k;
    }
}

int* Sudocube::getRedCase() {
    return redCase;
}

int Sudocube::getRedCaseValue() {
    return getCaseValue(redCase[0], redCase[1], redCase[2]);
}

void Sudocube::removePossibility(int i, int j, int k, int value) {
    if (indexesOk(i, j, k)) {
        container[i - 1][j - 1][k - 1]->removePossibility(value);
    }
}

vector<int> Sudocube::getPossibilities(int i, int j, int k) {
    vector<int> listOfPossibilities;
    if (indexesOk(i, j, k)) {
        listOfPossibilities = container[i - 1][j - 1][k - 1]->getPossibilities();
    }
    return listOfPossibilities;
}

void Sudocube::setPossibilities(int i, int j, int k, vector<int> possibilities) {
    if (indexesOk(i, j, k)) {
        container[i - 1][j - 1][k - 1]->setPossibilities(possibilities);
    }
}

bool Sudocube::indexesOk(int i, int j, int k) {
    if (i >= 1 && i <= 3 && j >= 1 && j <= 4 && k >= 1 && k <= 4) {
        return true;
    }
    return false;
}

string Sudocube::print() {
    stringstream buff;
    for (int k = 3; k >= 0; k--) {
        buff << "     ";
        for (int j = 0; j < 4; j++) {
            buff << container[2][j][k]->getValue() << " ";
        }
        buff << endl;
    }
    for (int x = 3; x >= 0; x--) {
        buff << container[0][x][3]->getValue() << " ";
        buff << container[0][x][2]->getValue() << " ";
        buff << container[0][x][1]->getValue() << " ";
        buff << container[0][x][0]->getValue() << "   ";
        buff << container[1][0][x]->getValue() << " ";
        buff << container[1][1][x]->getValue() << " ";
        buff << container[1][2][x]->getValue() << " ";
        buff << container[1][3][x]->getValue() << " ";
        buff << endl;
    }

    return buff.str();
}

vector<vector<Case*> > Sudocube::getListOfAllCaseLinesAndRegions() {
    vector<vector<Case*> > allRegions;

    allRegions.push_back(getSameLineOfCase(1, 1, 4));
    allRegions.push_back(getSameLineOfCase(1, 1, 3));
    allRegions.push_back(getSameLineOfCase(1, 1, 2));
    allRegions.push_back(getSameLineOfCase(1, 1, 1));
    allRegions.push_back(getSameColumnOfCase(1, 1, 1));
    allRegions.push_back(getSameColumnOfCase(1, 2, 1));
    allRegions.push_back(getSameColumnOfCase(1, 3, 1));
    allRegions.push_back(getSameColumnOfCase(1, 4, 1));
    allRegions.push_back(getSameColumnOfCase(2, 1, 1));
    allRegions.push_back(getSameColumnOfCase(2, 2, 1));
    allRegions.push_back(getSameColumnOfCase(2, 3, 1));
    allRegions.push_back(getSameColumnOfCase(2, 4, 1));
    allRegions.push_back(getSameRegionOfCase(1, 1, 1));
    allRegions.push_back(getSameRegionOfCase(1, 3, 1));
    allRegions.push_back(getSameRegionOfCase(2, 1, 1));
    allRegions.push_back(getSameRegionOfCase(2, 3, 1));
    allRegions.push_back(getSameRegionOfCase(3, 1, 1));
    allRegions.push_back(getSameRegionOfCase(3, 3, 1));

    return allRegions;
}

vector<vector<Case*> > Sudocube::getListOfAllCaseLines() {
    vector<vector<Case*> > allColumns;

    allColumns.push_back(getSameLineOfCase(1, 1, 1));
    allColumns.push_back(getSameLineOfCase(1, 1, 2));
    allColumns.push_back(getSameLineOfCase(1, 1, 3));
    allColumns.push_back(getSameLineOfCase(1, 1, 4));
    allColumns.push_back(getSameColumnOfCase(2, 1, 1));
    allColumns.push_back(getSameColumnOfCase(2, 2, 1));
    allColumns.push_back(getSameColumnOfCase(2, 3, 1));
    allColumns.push_back(getSameColumnOfCase(2, 4, 1));
    allColumns.push_back(getSameColumnOfCase(1, 1, 1));
    allColumns.push_back(getSameColumnOfCase(1, 2, 1));
    allColumns.push_back(getSameColumnOfCase(1, 3, 1));
    allColumns.push_back(getSameColumnOfCase(1, 4, 1));

    return allColumns;
}

vector<Case*> Sudocube::getSameLineOfCase(int i, int j, int k) {
    vector<Case*> sameLineCases;

    sameLineCases.push_back(container[i - 1][0][k - 1]);
    sameLineCases.push_back(container[i - 1][1][k - 1]);
    sameLineCases.push_back(container[i - 1][2][k - 1]);
    sameLineCases.push_back(container[i - 1][3][k - 1]);

    int x;
    int y;
    if (i == 1) {
        x = 2;
        y = 4 - k;
    } else if (i == 2) {
        x = 0;
        y = k - 1;
    } else { //i == 3
        x = 1;
        y = k - 1;
    }

    sameLineCases.push_back(container[x][y][0]);
    sameLineCases.push_back(container[x][y][1]);
    sameLineCases.push_back(container[x][y][2]);
    sameLineCases.push_back(container[x][y][3]);

    return sameLineCases;
}

vector<Case*> Sudocube::getSameColumnOfCase(int i, int j, int k) {
    vector<Case*> sameColumnCases;
    sameColumnCases.push_back(container[i - 1][j - 1][0]);
    sameColumnCases.push_back(container[i - 1][j - 1][1]);
    sameColumnCases.push_back(container[i - 1][j - 1][2]);
    sameColumnCases.push_back(container[i - 1][j - 1][3]);

    int x;
    int z;
    if (i == 1) {
        x = 1;
        z = j - 1;
    } else if (i == 2) {
        x = 2;
        z = j - 1;
    } else { //i == 3
        x = 0;
        z = 4 - j;
    }
    sameColumnCases.push_back(container[x][0][z]);
    sameColumnCases.push_back(container[x][1][z]);
    sameColumnCases.push_back(container[x][2][z]);
    sameColumnCases.push_back(container[x][3][z]);

    return sameColumnCases;
}

vector<Case*> Sudocube::getSameRegionOfCase(int i, int j, int k) {
    vector<Case*> sameRegionCases;
    if (j <= 2) {
        sameRegionCases.push_back(container[i - 1][0][0]);
        sameRegionCases.push_back(container[i - 1][0][1]);
        sameRegionCases.push_back(container[i - 1][0][2]);
        sameRegionCases.push_back(container[i - 1][0][3]);
        sameRegionCases.push_back(container[i - 1][1][0]);
        sameRegionCases.push_back(container[i - 1][1][1]);
        sameRegionCases.push_back(container[i - 1][1][2]);
        sameRegionCases.push_back(container[i - 1][1][3]);
    } else {
        sameRegionCases.push_back(container[i - 1][2][0]);
        sameRegionCases.push_back(container[i - 1][2][1]);
        sameRegionCases.push_back(container[i - 1][2][2]);
        sameRegionCases.push_back(container[i - 1][2][3]);
        sameRegionCases.push_back(container[i - 1][3][0]);
        sameRegionCases.push_back(container[i - 1][3][1]);
        sameRegionCases.push_back(container[i - 1][3][2]);
        sameRegionCases.push_back(container[i - 1][3][3]);
    }
    return sameRegionCases;
}
