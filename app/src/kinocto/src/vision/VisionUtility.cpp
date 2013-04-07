#include "vision/VisionUtility.h"

using namespace cv;
using namespace std;

void VisionUtility::applyErode(Mat & toErode, const int size, const int morphShape) {
    Point erodePoint(size, size);
    Mat erodeElem = getStructuringElement(morphShape, Size(2 * size + 1, 2 * size + 1), erodePoint);
    erode(toErode, toErode, erodeElem);
}

void VisionUtility::applyDilate(Mat & toDilate, const int size, const int morphShape) {
    Point dilatePoint(size, size);
    Mat dilateElem = getStructuringElement(morphShape, Size(2 * size + 1, 2 * size + 1), dilatePoint);
    dilate(toDilate, toDilate, dilateElem);
}

void VisionUtility::saveImage(Mat &pict, char* filename) {
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    imwrite(filename, pict, compression_params);
}
