//TODO : Faire des tests avec plusieurs positions de robot

#include "ObjectDetector.h"
#include "KinectTransformator.h"

const float ObjectDetector::TABLE_WIDTH = 1.10f;

struct SortByXY {
    bool operator() (Rect const & L, Rect const & R) {
        if (L.x < R.x){
            return true;
        }
        else if (L.x == R.x && L.y < R.y){
            return true;
        }
        else{
            return false;
        }
    }
};

Vec2f ObjectDetector::getAverageDistanceForPointLine(list<Vec2f> allDistances) {
    float averageXPosition = 0;
    int countAverageX = 0;
    float averageZPosition = 0;
    int countAverageZ = 0;

    if (allDistances.size() > 0) {
        for (int i = 0; i < allDistances.size() - 1; i++) {
            Vec2f distance = allDistances.front();
            allDistances.pop_front();
            if (abs(distance[0] / allDistances.front()[0]) >= 0.95 && distance[0] <= allDistances.front()[0]) {
                averageXPosition += distance[0];
                countAverageX++;
            }
            else if (abs(distance[0] / allDistances.front()[0]) <= 1.05 && distance[0] >= allDistances.front()[0]) {
                averageXPosition += distance[0];
                countAverageX++;
            }

            if (abs(distance[1] / allDistances.front()[1]) >= 0.95 && distance[1] <= allDistances.front()[1]) {
                averageZPosition += distance[1];
                countAverageZ++;
            }
            else if (abs(distance[1] / allDistances.front()[1]) <= 1.05 && distance[1] >= allDistances.front()[1]) {
                averageZPosition += distance[1];
                countAverageZ++;
            }
        }
    }

    if (countAverageX > 0)
        averageXPosition /= countAverageX;
    if (countAverageZ > 0)
        averageZPosition /= countAverageZ;

    return Vec2f(averageXPosition, averageZPosition);
}

int ObjectDetector::getAverageFromPointList(list<Point> obstacle) {
    int averagePointObstacle = 0;
    int obstacleSize = obstacle.size();

    for (int i = 0; i < obstacleSize; i++) {
        averagePointObstacle += obstacle.front().x;
        obstacle.pop_front();
    }

    if (obstacleSize > 0) {
        averagePointObstacle /= obstacleSize;
        return averagePointObstacle;
    }
    else {
        return 0;
    }
}

int ObjectDetector::generateQuads(Mat &picture, vector<Rect>&outQuads){
    int minSize = 25;
    int maxContourApprox = 7;
    Mat RGB = picture.clone();
    Mat RGBGray;
    
    vector<Point> srcContour;
    
    vector<vector<Point> > frameContours;
    vector<Vec4i> frameHierarchy;

    cvtColor(picture, RGBGray, CV_RGB2GRAY);
    
    Canny(RGBGray,RGBGray, 50,200, 3);
    findContours(RGBGray, frameContours, frameHierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    
    // get all the contours one by one
    for (int j=0; j<frameContours.size(); j++) {
        srcContour = frameContours[j];
        
        vector<Point> dstContour;
        Rect rect = boundingRect(srcContour);
        
        // reject contours with too small perimeter
        if(rect.width*rect.height >= minSize)
        {
            for(int approxLevel = 1; approxLevel <= maxContourApprox; approxLevel++ )
            {
                approxPolyDP(srcContour, dstContour, 3, true);
                if( dstContour.size() == 4 )
                    break;
                
                // we call this again on its own output, because sometimes
                // cvApproxPoly() does not simplify as much as it should.
                approxPolyDP(dstContour, dstContour, 3, true);
                if( dstContour.size() == 4 )
                    break;
            }
            
            // reject non-quadrangles
            if( dstContour.size() == 4 && isContourConvex(dstContour) )
            {
                Rect rect2 = boundingRect(Mat(dstContour));
                
                double d1, d2;
                double p = rect2.width*2+rect2.height*2;
                double area = fabs((float)rect2.area());
                double dx, dy;
                
                
                dx = dstContour[0].x - dstContour[2].x;
                dy = dstContour[0].y - dstContour[2].y;
                d1 = sqrt(dx*dx + dy*dy);
                
                dx = dstContour[1].x - dstContour[3].x;
                dy = dstContour[1].y - dstContour[3].y;
                d2 = sqrt(dx*dx + dy*dy);

                double d3, d4;
                dx = dstContour[0].x - dstContour[1].x;
                dy = dstContour[0].y - dstContour[1].y;
                d3 = sqrt(dx*dx + dy*dy);
                dx = dstContour[1].x - dstContour[2].x;
                dy = dstContour[1].y - dstContour[2].y;
                d4 = sqrt(dx*dx + dy*dy);
                if((d3*4 > d4 && d4*4 > d3 && d3*d4 < area*1.5 && area > minSize &&
                    d1 >= 0.15 * p && d2 >= 0.15 * p) )
                {
                    if (outQuads.size() == 0){
                        outQuads.push_back(rect2);
                    }
                    else{
                        for (int k = 0; k < outQuads.size(); k++){
                            Rect quad = outQuads[k];
                            if(quad.x != rect2.x && quad.y != rect2.y && quad.width != rect2.width && quad.height != rect2.height){
                                outQuads.push_back(rect2);
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    
    
    removeDoubleSquare(outQuads);
    
    removeQuadsNotOnChessboard(outQuads);

    sortQuadsByPosition(outQuads);

    return outQuads.size();
}

int ObjectDetector::removeDoubleSquare(vector<Rect> &outQuads){
    float acceptablePercent = 0.70f;
    vector<Rect> tempList;

    for(int i = 0; i< outQuads.size(); i++){
        bool singleSquare = true;
        for(int j = 0; j < outQuads.size(); j++){
            if(i != j){
                Rect interesect  = outQuads[i] & outQuads[j];
                float rectArea1 = outQuads[j].width * outQuads[j].height;
                float intersectArea = interesect.width * interesect.height;

                if(intersectArea/rectArea1 > acceptablePercent && intersectArea != 0)
                {                    
                    singleSquare = false;
                    break;
                }
            }
        }

        if(singleSquare){
            tempList.push_back(outQuads[i]);
        }
    }

    outQuads = tempList;
    return outQuads.size();
}

int ObjectDetector::removeQuadsNotOnChessboard(vector<Rect> &outQuads){
    int maxRange = 10;
    vector<Rect> tempList;
    for(int i = 0; i< outQuads.size(); i++){
        int neighborsCount = 0;
        Rect acceptableRange(outQuads[i].x - maxRange, outQuads[i].y - maxRange, outQuads[i].width+(2 * maxRange), outQuads[i].height+(2 * maxRange));
        for(int j = 0; j < outQuads.size(); j++){
            if (i != j){
               Rect tempQuad = outQuads[j];

                //Verify if the outer cube is in the accepted perimeter of the other one
                //TODO: Find a better way to do that
                if (tempQuad.x < (acceptableRange.x + acceptableRange.width) &&
                        tempQuad.x > acceptableRange.x &&
                        tempQuad.y < (acceptableRange.y + acceptableRange.height) &&
                        tempQuad.y > acceptableRange.y) {
                    neighborsCount++;
                } else if (tempQuad.x + tempQuad.width < (acceptableRange.x + acceptableRange.width) &&
                        tempQuad.x + tempQuad.width > acceptableRange.x &&
                        tempQuad.y < (acceptableRange.y + acceptableRange.height) &&
                        tempQuad.y > acceptableRange.y) {
                    neighborsCount++;
                } else if (tempQuad.x < (acceptableRange.x + acceptableRange.width) &&
                        tempQuad.x > acceptableRange.x &&
                        tempQuad.y + tempQuad.height < (acceptableRange.y + acceptableRange.height) &&
                        tempQuad.y + tempQuad.height > acceptableRange.y) {
                    neighborsCount++;
                } else if (tempQuad.x + tempQuad.width < (acceptableRange.x + acceptableRange.width) &&
                        tempQuad.x + tempQuad.width > acceptableRange.x &&
                        tempQuad.y + tempQuad.height < (acceptableRange.y + acceptableRange.height) &&
                        tempQuad.y + tempQuad.height > acceptableRange.y) {
                    neighborsCount++;
                }

                if (neighborsCount >= 3){
                    tempList.push_back(outQuads[i]);
                    break;
                }
            }
        }
    }

    outQuads = tempList;
    return tempList.size();
}

void ObjectDetector::sortQuadsByPosition(vector<Rect> &outQuads){
    sort(outQuads.begin(), outQuads.end(), SortByXY());
}

ObjectDetector::quadColor ObjectDetector::findQuadColor( Mat &picture, const vector<Rect> &squares )
{
    Mat HSVPicture;
    cvtColor(picture, HSVPicture, CV_BGR2HSV);
    
    int coloredSquareCount = 0;

    for(int i = 0; i < squares.size(); i++){
        Rect square = squares[i];
        cv::Vec3b pixel = HSVPicture.at<Vec3b>(square.x + (square.width / 2), square.y + (square.height / 2));

        if(pixel[2] > 30 && pixel[0] > 100 && pixel[0] < 160){
            coloredSquareCount++;
        }
    }

    if(coloredSquareCount > squares.size() * 0.5){
        return quadColor::BLUE;
    }
    else{
        return quadColor::BLACK;
    }
}
