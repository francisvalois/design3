/* 
 * File:   main.cpp
 * Author: mouhtij
 *
 * Created on 21 février 2013, 00:52
 */

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include<math.h>
#include <iostream>

using namespace cv;
using namespace std;

Mat src;
Point startPt(0,0);
Point endPt(0,0);
float dist;
/*      Void distanceObstacle(){
 *      Distance entre les obstacles
 *      double x1, x2, y1. y2; // (x1,y1) coordonnees objet1, (x2, y2) coordonnees objet2
 *      int XposKinect =0;
 *      int YposKinect =0;
 *      double teta1; 
 *      double teta2; 
 *      double tetaFinal; 
 *      double dist1 = x1^2+y1^2;
 *      double dist2 = x2^2+y2^2
 *      teta1= sin(y1)/sqrt(dist1);
 *      teta2= sin(y2)/sqrt(dist2);
 *      tetaFinal=teta2-teta1;
 *      }
 *  
 *      
 */

void onMouse( int event, int x, int y, int flags, void* )
{
    if( event == CV_EVENT_LBUTTONUP) startPt = Point(x,y);
    if( event == CV_EVENT_RBUTTONUP) {
        endPt   = Point(x,y);
        Vec3f s = src.at<Vec3f>(startPt.y, startPt.x);
        Vec3f e = src.at<Vec3f>(endPt.y, endPt.x);
        float dx = e[0]-s[0];
        float dy = e[1]-s[1];
        float dz = e[2]-s[2];
        dist = sqrt(dx*dx + dy*dy + dz*dz);
    }
}

int main( /*int argc, char* argv[]*/ ){
    VideoCapture capture;
    capture.open(CV_CAP_OPENNI);
    capture.set( CV_CAP_PROP_OPENNI_REGISTRATION , 0);
    unsigned t0=clock();
    if( !capture.isOpened() ){
        cout << "Can not open a capture object." << endl;
        return -1;
    }
    unsigned elapsed=clock()-t0;
    cout << "initialized in "<< elapsed <<" s. ready!" << endl;
    namedWindow( "depth", 1 );
    setMouseCallback( "depth", onMouse, 0 );
    for(;;){
        if( !capture.grab() ){
            cout << "Can not grab images." << endl;
            return -1;
        }else{
            Mat depthMap,show;
            capture.retrieve(src, CV_CAP_OPENNI_POINT_CLOUD_MAP);
            if( capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP ) ) depthMap.convertTo( show, CV_8UC1, 0.05f);
            line(show,startPt,endPt,Scalar(255));
            putText(show,format("distance: %f m",dist),Point(5,15),FONT_HERSHEY_PLAIN,1,Scalar(255));
            imshow("depth",show);
        }
        if( waitKey( 30 ) >= 0 )    break;
    }    }
