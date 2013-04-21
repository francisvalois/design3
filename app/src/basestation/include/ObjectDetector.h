#ifndef __object_detector_
#define __object_detector_

#include <iostream>
#include <list>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"

#include "KinectTransformator.h"

class ObjectDetector {
public:
    int generateQuads(cv::Mat &image, std::vector<cv::Rect>&outQuads, bool applyCorrection = true);

protected:
    static float const TABLE_WIDTH;

    int getAverageFromPointList(std::list<cv::Point> obstacle);

    cv::Vec2f getAverageDistanceForPointLine(std::list<cv::Vec2f> allDistances);

    int removeQuadsNotOnChessboard(std::vector<cv::Rect> & outQuads);

    void sortQuadsByPosition(std::vector<cv::Rect> & outQuads);

    int removeDoubleSquare(std::vector<cv::Rect> & outQuads);

    std::vector<cv::Rect> removeOutBoundsFrameRect(const cv::Mat& depthMap,
            std::vector<cv::Rect> frameRect);

    cv::Mat segmentBlueFrame(const cv::Mat & img);

    cv::vector<cv::Rect> getFrameRect(const cv::Mat & img);
};

#endif
