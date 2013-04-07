#ifndef __kinect_detection_
#define __kinect_detection_

#include <iostream>
#include "opencv2/core/core.hpp"
#include <list>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"


using namespace cv;
using namespace std;

class ObjectDetector {

protected:
    static float const TABLE_WIDTH;

    int getAverageFromPointList(list<Point> obstacle);
    Vec2f getAverageDistanceForPointLine(list<Vec2f> allDistances);
    int removeQuadsNotOnChessboard(vector<Rect>  & outQuads);
    void sortQuadsByPosition(vector<Rect>  & outQuads);
    int removeDoubleSquare(vector<Rect>  & outQuads);

public:
    int generateQuads(Mat &image, vector<Rect>&outQuads);

};

#endif //__kinect_H_
