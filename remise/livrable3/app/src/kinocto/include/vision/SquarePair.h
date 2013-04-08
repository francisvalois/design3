#ifndef SQUAREPAIR_H_
#define SQUAREPAIR_H_

#include "opencv2/core/core.hpp"

class SquarePair {

public:
    SquarePair() {
    }
    SquarePair(cv::Rect rect, std::vector<cv::Point> poly) {
        this->rect = rect;
        this->poly = poly;
    }
    cv::Rect rect;
    std::vector<cv::Point> poly;
};

#endif /* SQUAREPAIR_H_ */
