//
// Created by francisvalois on 2013-02-28.
//
// To change the template use AppCode | Preferences | File Templates.
//



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
