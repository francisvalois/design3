#ifndef CASE_H_
#define CASE_H_

#include <iostream>
#include <vector>

class Case {
public:
    Case();
    Case(int, int, int);
    virtual ~Case();

    std::vector<int> getPossibilities();
    void setPossibilities(std::vector<int>);
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
    std::vector<int> possibilities;
    int i;
    int j;
    int k;
};

#endif /* CASE_H_ */
