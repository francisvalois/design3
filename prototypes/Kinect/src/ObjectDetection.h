#ifndef __kinect_detection_
#define __kinect_detection_

#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <list>


using namespace cv;
using namespace std;

class ObjectDetection {

protected:
    static float const TABLE_WIDTH;

    int getAverageFromPointList(list<Point> obstacle);
    Vec2f getAverageDistanceForPointLine(list<Vec2f> allDistances);
    int getAverageFromPointListWithConditions(vector<Point> positions, float minCondition, float maxCondition);
public:
    
};

#endif //__kinect_H_
