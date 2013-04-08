#ifndef SQUARESPAIRSORTER_H_
#define SQUARESPAIRSORTER_H_

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "vision/SquarePair.h"

class SquaresPairSorter {

public:
    SquaresPairSorter();
    virtual ~SquaresPairSorter();

    std::vector<std::vector<SquarePair> > sortSquaresPair(std::vector<SquarePair> squaresRect, const int frameWidth);

private:
};

#endif
