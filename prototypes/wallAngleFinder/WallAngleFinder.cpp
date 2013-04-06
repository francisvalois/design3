#include "WallAngleFinder.h"

WallAngleFinder::WallAngleFinder() {
}

WallAngleFinder::~WallAngleFinder() {
}

void WallAngleFinder::applyErode(Mat & toErode, int size, int morphShape) {
    Point erodePoint(size, size);
    Mat erodeElem = getStructuringElement(morphShape, Size(2 * size + 1, 2 * size + 1), erodePoint);
    erode(toErode, toErode, erodeElem);
}

double WallAngleFinder::calculateAngleFrom (Point2d * first, Point2d * last) {
    double xComp = last->x - first->x;
    double yComp = last->y - first->y;
    
    double hypo = sqrt(xComp * xComp + yComp * yComp);
    double angle = asin(yComp / hypo) * 180.0 / CV_PI;

    return angle;
}

vector<Point2d * > WallAngleFinder::findSlopePoints(Mat & wall) {
    vector<Point2d *> points;
    for (int i = 0; i < wall.cols; i += STEP_SIZE) {
        for (int j = 0; j < wall.rows; j++) {
            if (wall.at<uchar>(j,i) == 250) {
                points.push_back(new Point2d(i,j));
                break;
            }
        }
    }
    
    return points;
}

double WallAngleFinder::calculateSlopeAverage(vector<Point2d * > points, int nbOfStep) {
    double moy = 0;
    for (int i = 0; i < (nbOfStep/ 2); i++) {
        Point2d * first = points[i];
        Point2d * last = points[nbOfStep - i- 1];
        double angle = calculateAngleFrom (first, last);
        moy += (angle / (nbOfStep/ 2));
    }
    
    return moy;
}

double WallAngleFinder::findAngle(Mat & wall) {
    threshold(wall, wall, 100, 250, THRESH_BINARY);
	applyErode(wall, 7, MORPH_ELLIPSE);

    int nbOfStep = wall.cols / STEP_SIZE;
    
    vector<Point2d *> points = findSlopePoints(wall);
    
    return calculateSlopeAverage(points, nbOfStep);
}

int main(int, char**) {
	Mat wall = imread("moin15degree.png", CV_LOAD_IMAGE_GRAYSCALE);
    WallAngleFinder anglefinder;
    
    cout << "angle:" << anglefinder.findAngle(wall) << endl;
	
	return 0;
}
