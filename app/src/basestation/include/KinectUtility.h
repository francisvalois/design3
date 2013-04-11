#ifndef __KinectUtility_H_
#define __KinectUtility_H_

#include <iostream>
#include <string>

#include "opencv2/core/core.hpp"

class Utility {
public:
    static void saveToFile(cv::Mat matrix, std::string fileName);
    static cv::Mat readFromFile(std::string fileName);
};

#endif
