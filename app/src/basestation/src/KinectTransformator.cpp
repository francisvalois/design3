#include "KinectTransformator.h"
#define _USE_MATH_DEFINES
#include <math.h>


using namespace cv;
using namespace std;

const float KinectTransformator::KINECTANGLE = 22.5f;
float KinectTransformator::_kinectAngleRad = (float) (KINECTANGLE / 180 * M_PI);
Vec2f KinectTransformator::_kinectPosition = Vec2f(0.135f, -0.53f);

void KinectTransformator::setKinectAngle(float angleRad){
    if(angleRad > 0 && angleRad < M_PI){
        _kinectAngleRad = angleRad;
    }
}

Vec2f KinectTransformator::getKinectPosition(){
    return _kinectPosition;
}

float KinectTransformator::getKinectAngle(){
    return _kinectAngleRad;
}

void KinectTransformator::setKinectPosition(Vec2f kinectPosition){
     _kinectPosition = kinectPosition;
}

Vec2f KinectTransformator::getRotatedXZCoordFromKinectCoord(Vec3f depthXYZ) {
    float depthZ = depthXYZ[2];
    float depthX = depthXYZ[0];
    float trueDepthX = sin(_kinectAngleRad) * depthZ - cos(_kinectAngleRad) * depthX;
    float trueDepthZ = sin(_kinectAngleRad) * depthX + cos(_kinectAngleRad) * depthZ;
    Vec2f trueDepth(trueDepthX, trueDepthZ);

    return trueDepth;
}

Vec2f KinectTransformator::translateXZCoordtoOrigin(Vec2f rotatedXZ) {
    float positionZ = rotatedXZ[1] + _kinectPosition[1];
    float positionX = rotatedXZ[0] + _kinectPosition[0];
    Vec2f modifiedXZPosition(positionX, positionZ);
    
    return modifiedXZPosition;
}


Vec2f KinectTransformator::getTrueCoordFromKinectCoord(Vec3f depthXYZ) {
    if(depthXYZ[2] < 0.5)
    {
        return Vec2f();
    }
    
    Vec2f rotPosition = getRotatedXZCoordFromKinectCoord(depthXYZ);
    Vec2f realPosition = translateXZCoordtoOrigin(rotPosition);
    return realPosition;
}



Point KinectTransformator::findNeareastValueInLookupTable(int expectedValueX, int expectedValueY){
    float B22data[] = {1, 2, 3, 4};
    Mat B22 = Mat(2, 2, CV_32F, B22data).clone();
    cout << B22 << endl;

    vector<float> test3 = B22.row(0);

    cout << test3[0] << endl;

    struct {
        int operator()(int a, int b) {return a > b;}
    } comparator;

    //std::vector<int>::iterator it = std::find_if (test3.begin(), test3.end(), comparator);
    //std::cout << "The first odd value is " << *it << '\n';

    std::vector<float>::iterator low;
    low=std::lower_bound (test3.begin(), test3.end(), expectedValueY);

    float beforeValue = low - test3.begin();






    cout << low - test3.begin() << endl;
}
