#ifndef CASE_H_
#define CASE_H_

#include <iostream>
#include <vector>

using namespace std;

class Case {
public:
    Case();
    Case(int, int, int);
    virtual ~Case();

    vector<int> getPossibilities();
    void setPossibilities(vector<int>);
    void removePossibility(int);
    int numberOfPossibilitiesRemaining();
    bool isSolved();

    bool containsValue(int);
    int getValue();
    void setValue(int);

    int getI() {
        return i;
    };
    int getJ() {
        return j;
    };
    int getK() {
        return k;
    };

private:
    vector<int> possibilities;
    int i;
    int j;
    int k;
};

#endif /* CASE_H_ */
