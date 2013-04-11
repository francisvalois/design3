#ifndef __object_detector_
#define __object_detector_

#include <iostream>
#include <list>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

#include "KinectTransformator.h"

class ObjectDetector {
public:
    int generateQuads(cv::Mat &image, std::vector<cv::Rect>&outQuads);
    enum quadColor {
        BLUE, BLACK, RED
    };

protected:
    static float const TABLE_WIDTH;

    int getAverageFromPointList(std::list<cv::Point> obstacle);
    cv::Vec2f getAverageDistanceForPointLine(std::list<cv::Vec2f> allDistances);
    int removeQuadsNotOnChessboard(std::vector<cv::Rect> & outQuads);
    void sortQuadsByPosition(std::vector<cv::Rect> & outQuads);
    int removeDoubleSquare(std::vector<cv::Rect> & outQuads);
    quadColor findQuadColor(cv::Mat &image, const std::vector<cv::Rect> &squares);
};

#endif
