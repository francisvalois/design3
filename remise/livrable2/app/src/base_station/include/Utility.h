#ifndef __Utility_H_
#define __Utility_H_

#include <iostream>
#include "opencv2/core/core.hpp"

using namespace cv;

class Utility {
public:
    static void saveToFile(Mat matrix, string fileName);
    static Mat readFromFile(string fileName);
};

#endif //__Utility_H_
