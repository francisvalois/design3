#ifndef VISIONUTILITY_H_
#define VISIONUTILITY_H_

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class VisionUtility {

private:

public:
    static void applyErode(cv::Mat & toErode, int size, int morphShape);
    static void applyDilate(cv::Mat & toDilate, int size, int morphShape);
    static void saveImage(cv::Mat &pict, char* filename);
};

#endif
