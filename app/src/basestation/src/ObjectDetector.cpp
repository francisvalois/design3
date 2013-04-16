#include "ObjectDetector.h"

using namespace cv;
using namespace std;

const float ObjectDetector::TABLE_WIDTH = 1.10f;

struct SortByXY {
    bool operator()(Rect const & L, Rect const & R) {
        if (L.x < R.x) {
            return true;
        } else if (L.x == R.x && L.y < R.y) {
            return true;
        } else {
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
            } else if (abs(distance[0] / allDistances.front()[0]) <= 1.05 && distance[0] >= allDistances.front()[0]) {
                averageXPosition += distance[0];
                countAverageX++;
            }

            if (abs(distance[1] / allDistances.front()[1]) >= 0.95 && distance[1] <= allDistances.front()[1]) {
                averageZPosition += distance[1];
                countAverageZ++;
            } else if (abs(distance[1] / allDistances.front()[1]) <= 1.05 && distance[1] >= allDistances.front()[1]) {
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

Rect ObjectDetector::getQuadEnglobingOthers(vector<Rect> quads) {
    int minX = 0, minY = 0, maxX = 0, maxY = 0;

    if(quads.size() > 0){
        minX = quads[0].x;
        minY = quads[0].y;
        maxX = quads[0].x + quads[0].width;
        maxY = quads[0].y + quads[0].height;

        for(int i = 1; i < quads.size(); i++){
            if(quads[i].x < minX)
                minX = quads[i].x;
            if(quads[i].y < minY)
                minY = quads[i].y;
            if(quads[i].x + quads[i].width > maxX)
                maxX = quads[i].x + quads[i].width;
            if(quads[i].y + quads[i].height > maxY)
                maxY = quads[i].y + quads[i].height;
        }
    }

    return Rect(Point(minX, minY), Point(maxX, maxY));
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
    } else {
        return 0;
    }
}

int ObjectDetector::generateQuads(Mat &picture, vector<Rect>&outQuads, bool applyCorrection) {
    int minSize = 25;
    int maxContourApprox = 7;
    Mat RGB = picture.clone();
    Mat RGBGray;

    vector<Point> srcContour;

    vector<vector<Point> > frameContours;
    vector<Vec4i> frameHierarchy;

    if(picture.channels() == 1){
        RGBGray = picture.clone();
    }
    else{
        cvtColor(picture, RGBGray, CV_RGB2GRAY);
    }

    Canny(RGBGray, RGBGray, 50, 200, 3);
    findContours(RGBGray, frameContours, frameHierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    // get all the contours one by one
    for (int j = 0; j < frameContours.size(); j++) {
        srcContour = frameContours[j];

        vector<Point> dstContour;
        Rect rect = boundingRect(srcContour);

        // reject contours with too small perimeter
        if (rect.width * rect.height >= minSize) {
            for (int approxLevel = 1; approxLevel <= maxContourApprox; approxLevel++) {
                approxPolyDP(srcContour, dstContour, 3, true);
                if (dstContour.size() == 4)
                    break;

                // we call this again on its own output, because sometimes
                // cvApproxPoly() does not simplify as much as it should.
                approxPolyDP(dstContour, dstContour, 3, true);
                if (dstContour.size() == 4)
                    break;
            }

            // reject non-quadrangles
            if (dstContour.size() == 4 && isContourConvex(dstContour)) {
                Rect rect2 = boundingRect(Mat(dstContour));

                double d1, d2;
                double p = rect2.width * 2 + rect2.height * 2;
                double area = fabs((float) rect2.area());
                double dx, dy;

                dx = dstContour[0].x - dstContour[2].x;
                dy = dstContour[0].y - dstContour[2].y;
                d1 = sqrt(dx * dx + dy * dy);

                dx = dstContour[1].x - dstContour[3].x;
                dy = dstContour[1].y - dstContour[3].y;
                d2 = sqrt(dx * dx + dy * dy);

                double d3, d4;
                dx = dstContour[0].x - dstContour[1].x;
                dy = dstContour[0].y - dstContour[1].y;
                d3 = sqrt(dx * dx + dy * dy);
                dx = dstContour[1].x - dstContour[2].x;
                dy = dstContour[1].y - dstContour[2].y;
                d4 = sqrt(dx * dx + dy * dy);
                if ((d3 * 4 > d4 && d4 * 4 > d3 && d3 * d4 < area * 1.5 && area > minSize && d1 >= 0.15 * p && d2 >= 0.15 * p)) {
                    if (outQuads.size() == 0) {
                        outQuads.push_back(rect2);
                    } else {
                        for (int k = 0; k < outQuads.size(); k++) {
                            Rect quad = outQuads[k];
                            if (quad.x != rect2.x && quad.y != rect2.y && quad.width != rect2.width && quad.height != rect2.height) {
                                outQuads.push_back(rect2);
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    if(applyCorrection){
        removeDoubleSquare(outQuads);

        removeQuadsNotOnChessboard(outQuads);

        sortQuadsByPosition(outQuads);
    }

    for (int i = 0; i < outQuads.size(); i++) {
        rectangle(RGB, outQuads[i], Scalar(0, 0, 255));
    }
    //imshow("debug", RGB);

    return outQuads.size();
}

int ObjectDetector::removeDoubleSquare(vector<Rect> &outQuads) {
    float acceptablePercent = 0.70f;
    vector<Rect> tempList;

    for (int i = 0; i < outQuads.size(); i++) {
        bool singleSquare = true;
        for (int j = 0; j < outQuads.size(); j++) {
            if (i != j) {
                Rect interesect = outQuads[i] & outQuads[j];
                float rectArea1 = outQuads[j].width * outQuads[j].height;
                float intersectArea = interesect.width * interesect.height;

                if (intersectArea / rectArea1 > acceptablePercent && intersectArea != 0) {
                    singleSquare = false;
                    break;
                }
            }
        }

        if (singleSquare) {
            tempList.push_back(outQuads[i]);
        }
    }

    outQuads = tempList;
    return outQuads.size();
}

bool ObjectDetector::containsRedSquares(Mat picture, Rect bigSquare) {
    Mat HSVPicture;
    cvtColor(picture, HSVPicture, CV_BGR2HSV);

    Mat croppedMat(HSVPicture, cv::Range(bigSquare.y, bigSquare.y + bigSquare.height),
            cv::Range(bigSquare.x, bigSquare.x + bigSquare.width));
    Mat HSVCroppedMat;
    cvtColor(croppedMat, HSVCroppedMat, CV_BGR2HSV);

    Mat segmentedRedSquare;
    Mat segmentedRedSquare2;
    inRange(HSVCroppedMat, Scalar(0, 220, 0), Scalar(25, 255, 200), segmentedRedSquare); // Pas le choix, en deux partie...
    inRange(HSVCroppedMat, Scalar(130, 220, 0), Scalar(255, 255, 200), segmentedRedSquare2);
    segmentedRedSquare += segmentedRedSquare2;

    vector<Rect> redSquares;
    bool applyCorrection = false;
    generateQuads(segmentedRedSquare, redSquares, applyCorrection);

    if(redSquares.size() >= 2){
        return true;
    }

    return false;
}

int ObjectDetector::removeQuadsNotOnChessboard(vector<Rect> &outQuads) {
    int maxRange = 10;
    vector<Rect> tempList;
    for (int i = 0; i < outQuads.size(); i++) {
        int neighborsCount = 0;
        Rect acceptableRange(outQuads[i].x - maxRange, outQuads[i].y - maxRange, outQuads[i].width + (2 * maxRange),
                outQuads[i].height + (2 * maxRange));
        for (int j = 0; j < outQuads.size(); j++) {
            if (i != j) {
                Rect tempQuad = outQuads[j];

                //Verify if the outer cube is in the accepted perimeter of the other one
                //TODO: Find a better way to do that
                if (tempQuad.x < (acceptableRange.x + acceptableRange.width) && tempQuad.x > acceptableRange.x
                        && tempQuad.y < (acceptableRange.y + acceptableRange.height) && tempQuad.y > acceptableRange.y) {
                    neighborsCount++;
                } else if (tempQuad.x + tempQuad.width < (acceptableRange.x + acceptableRange.width)
                        && tempQuad.x + tempQuad.width > acceptableRange.x && tempQuad.y < (acceptableRange.y + acceptableRange.height)
                        && tempQuad.y > acceptableRange.y) {
                    neighborsCount++;
                } else if (tempQuad.x < (acceptableRange.x + acceptableRange.width) && tempQuad.x > acceptableRange.x
                        && tempQuad.y + tempQuad.height < (acceptableRange.y + acceptableRange.height)
                        && tempQuad.y + tempQuad.height > acceptableRange.y) {
                    neighborsCount++;
                } else if (tempQuad.x + tempQuad.width < (acceptableRange.x + acceptableRange.width)
                        && tempQuad.x + tempQuad.width > acceptableRange.x
                        && tempQuad.y + tempQuad.height < (acceptableRange.y + acceptableRange.height)
                        && tempQuad.y + tempQuad.height > acceptableRange.y) {
                    neighborsCount++;
                }

                if (neighborsCount >= 3) {
                    tempList.push_back(outQuads[i]);
                    break;
                }
            }
        }
    }

    outQuads = tempList;
    return tempList.size();
}

void ObjectDetector::sortQuadsByPosition(vector<Rect> &outQuads) {
    sort(outQuads.begin(), outQuads.end(), SortByXY());
}

ObjectDetector::quadColor ObjectDetector::findQuadColor(Mat &picture, const vector<Rect> &squares) {
    Rect bigSquare = getQuadEnglobingOthers(squares);

    bool redSquare = containsRedSquares(picture, bigSquare);

    if (redSquare) {
        return RED;
    } else {
        return BLACK;
    }
}
