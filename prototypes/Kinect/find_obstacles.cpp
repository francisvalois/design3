//TODO : Refactor des fonctions parce que c'est affreux et ajout de la détection du rayon pour un position parfait des obstacles. De plus l'algorithme marche seulement si le robot n'est pas devant les obstacles. Je vais penser à une méthode pour arranger ça.

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <list>


using namespace cv;
using namespace std;

const float KINECTANGLE = (float)(25.0/360.0*2.0*M_PI);  //TODO : Trouver le vrai angle
const float X_KINECT_POSITION = 0.2f; //TODO : Trouver la vrai position de la Kinect
const float Z_KINECT_POSITION = -0.48f;
const int X_OBSTACLE_LEFT_THRESHOLD = 180;
const int X_OBSTACLE_RIGHT_THRESHOLD = 610;
const int Y_OBSTACLE_TOP_THRESHOLD = 80;
const int Y_OBSTACLE_BOTTOM_THRESHOLD = 272;
const float OBSTACLE_DISTANCE_MIN_THRESHOLD = 0.8f;
const float OBSTACLE_DISTANCE_MAX_THRESHOLD = 1.6F;

Mat world;
Point startPt(0,0);
Point endPt(0,0);

//Rotate depth Coords seen from the Kinect to align them with the the XYZ plane of the table
Vec2f getRotatedXYZCoordFromKinectCoord(Vec3f depthXYZ)
{
    float depthZ = depthXYZ[2];
    float depthX = depthXYZ[0];
    float trueDepthX = sin(KINECTANGLE)*depthZ - cos(KINECTANGLE)*depthX;
    float trueDepthZ = sin(KINECTANGLE)*depthX + cos(KINECTANGLE)*depthZ;
    Vec2f trueDepth(trueDepthX,trueDepthZ);
    
    return trueDepth;
}

Vec2f translateXYZCoordtoOrigin(Vec2f rotatedXZ)
{
    float positionZ = rotatedXZ[1] + Z_KINECT_POSITION;
    float positionX = rotatedXZ[0] + X_KINECT_POSITION;
    Vec2f modifiedXZPosition(positionX, positionZ);
    
    return modifiedXZPosition;
}

Vec2f getTrueCoordFromKinectCoord(Vec3f depthXYZ)
{
    Vec2f rotPosition = getRotatedXYZCoordFromKinectCoord(depthXYZ);
    Vec2f realPosition = translateXYZCoordtoOrigin(rotPosition);
    return realPosition;
}

Vec2f getAverageDistanceForObstacle(int obstaclePositionX, Mat depthMatrix)
{
    //Test different Y values to get an average for the distance and discard absurb values
    list<Vec2f> allDistances;
    for(int i = Y_OBSTACLE_TOP_THRESHOLD; i <= Y_OBSTACLE_BOTTOM_THRESHOLD; i += 25)
    {
        Vec3f kinectCoord = depthMatrix.at<Vec3f>(i, obstaclePositionX);
        Vec2f depthXYZ = getTrueCoordFromKinectCoord(kinectCoord);
        if(depthXYZ[1] > OBSTACLE_DISTANCE_MIN_THRESHOLD && depthXYZ[1] < OBSTACLE_DISTANCE_MAX_THRESHOLD){
            allDistances.push_back(Vec2f(depthXYZ[0], depthXYZ[1]));
        }
    }
    
    float averageXPosition = 0;
    int countAverageX = 0;
    float averageZPosition = 0;
    int countAverageZ = 0;
    
    for(int i=0; i < allDistances.size()-1; i++)
    {
        Vec2f distance = allDistances.front();
        allDistances.pop_front();
        Vec2f test = allDistances.front();
        if(abs(distance[0]/allDistances.front()[0]) >= 0.95 && distance[0] <= allDistances.front()[0])
        {
            averageXPosition += distance[0];
            countAverageX++;
        }
        else if(abs(distance[0]/allDistances.front()[0]) <= 1.05 && distance[0] >= allDistances.front()[0])
        {
            averageXPosition += distance[0];
            countAverageX++;
        }
        
        if(abs(distance[1]/allDistances.front()[1]) >= 0.95 && distance[1] <= allDistances.front()[1])
        {
            averageZPosition += distance[1];
            countAverageZ++;
        }
        else if(abs(distance[1]/allDistances.front()[1]) <= 1.05 && distance[1] >= allDistances.front()[1])
        {
            averageZPosition += distance[1];
            countAverageZ++;
        }
    }
    if(countAverageX > 0)
        averageXPosition /= countAverageX;
    if(countAverageZ > 0)
        averageZPosition /= countAverageZ;
    
    return Vec2f(averageXPosition, averageZPosition);
}

vector<Vec2f> findObstacles(Mat depthMatrix){
    Vec2f tempPosition;
    int count;
    int middleYPoint = (int)((Y_OBSTACLE_BOTTOM_THRESHOLD-Y_OBSTACLE_TOP_THRESHOLD) * 0.5f);
    list<Point> obstacle1;
    list<Point> obstacle2;
    list<Point> validObstaclePosition;
    Vec2f truePositionObstacle1;
    Vec2f truePositionObstacle2;
    
    //Find all point that could be an obstacle
    for(int i = X_OBSTACLE_LEFT_THRESHOLD; i <= X_OBSTACLE_RIGHT_THRESHOLD; i++)
    {
        count = 0;
        
        for (int j = Y_OBSTACLE_TOP_THRESHOLD; j <= Y_OBSTACLE_BOTTOM_THRESHOLD; j++) {
            tempPosition = getTrueCoordFromKinectCoord(depthMatrix.at<Vec3f>(j,i));
            if(tempPosition[1] > OBSTACLE_DISTANCE_MIN_THRESHOLD && tempPosition[1] < OBSTACLE_DISTANCE_MAX_THRESHOLD){
                count++;
            }
        }
        
        if(count >= middleYPoint)
        {
            if(validObstaclePosition.size() > 0)
            {
                if ((i - validObstaclePosition.back().x) > 10) // Separate first obstacle from second
                {
                    obstacle1 = validObstaclePosition;
                    validObstaclePosition.clear();
                }
            }
            validObstaclePosition.push_back(Point(i,middleYPoint));            
        }
    }
    
    if(validObstaclePosition.size() > 0 && obstacle1.size() > 0)
    {
        obstacle2 = validObstaclePosition;
    }
    else if(validObstaclePosition.size() > 0)
    {
        obstacle1 = validObstaclePosition;
    }
    
    int obstacle1PointCount = obstacle1.size();
    int obstacle2PointCount = obstacle2.size();

    if(obstacle1PointCount > 0){        
        float averagePointObstacle1 = 0;
        
        for (int i = 0; i < obstacle1PointCount; i++) {
            averagePointObstacle1 += obstacle1.front().x;
            obstacle1.pop_front();
        }
        averagePointObstacle1 /= obstacle1PointCount;
        truePositionObstacle1 = getAverageDistanceForObstacle(averagePointObstacle1, depthMatrix);
    }
    
    if(obstacle2PointCount > 0){
        float averagePointObstacle2 = 0;
        
        for (int i = 0; i < obstacle2PointCount; i++) {
            averagePointObstacle2 += obstacle2.front().x;
            obstacle2.pop_front();
        }
        averagePointObstacle2 /= obstacle2PointCount;
        truePositionObstacle2 = getAverageDistanceForObstacle(averagePointObstacle2, depthMatrix);
    }
        
    vector<Vec2f> obstaclesPosition;
    obstaclesPosition.push_back(truePositionObstacle1);
    obstaclesPosition.push_back(truePositionObstacle2);
    return obstaclesPosition;
}

void onMouse( int event, int x, int y, int flags, void* )
{
    if(event == CV_EVENT_LBUTTONUP){
        cout << "Pixel X :"<< x << "Pixel Y :" << y;
    }
    if( event == CV_EVENT_RBUTTONUP)
    {
        Vec3f s = world.at<Vec3f>(y, x);
        Vec2f rotPosition = getRotatedXYZCoordFromKinectCoord(s);
        Vec2f realPosition = translateXYZCoordtoOrigin(rotPosition);
        cout << "Position X :" << realPosition[0] << "Position Z:" << realPosition[1] << '\n';
    
    }
}

int main( /*int argc, char* argv[]*/ ){
    VideoCapture capture;
    Mat depthMap,show,showRGB;
    
    capture.open(CV_CAP_OPENNI);
    capture.set( CV_CAP_PROP_OPENNI_REGISTRATION , 0);

    if( !capture.isOpened() ){
        cout << "Can not open a capture object." << endl;
        return -1;
    }

    namedWindow( "depth", 1 );
    setMouseCallback( "depth", onMouse, 0 );
    
    capture.grab();
    capture.retrieve(world, CV_CAP_OPENNI_POINT_CLOUD_MAP);
    capture.retrieve(showRGB, CV_CAP_OPENNI_BGR_IMAGE);
    if(capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP))
        depthMap.convertTo( show, CV_8UC1, 0.05f);
    
    clock_t tStart = clock();
    
    vector<Vec2f> test = findObstacles(world);

    printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  
    imshow("depth",show);

    for(;;){
        if( !capture.grab() ){
            cout << "Can not grab images." << endl;
            return -1;
        }else{
            
        }
        if( waitKey( 30 ) >= 0 )    break;
    }
}