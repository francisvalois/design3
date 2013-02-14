#include "SudocubeExtractor.h"

using namespace std;
using namespace cv;

SudocubeExtractor::SudocubeExtractor() {
    white = Scalar(255, 255, 255);
}

bool SudocubeExtractor::isSudokuFrameVisible() {
    Mat src = camera.getImage();

    Mat srcHSV;
    cvtColor(src, srcHSV, CV_BGR2HSV);
    
    Mat greenFrameSegmented = segmentGreenFrame(srcHSV);
    
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(greenFrameSegmented, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    
    if (contours.size() != 2) {
        return false;
    } 
    
    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    for(int i = 0; i < contours.size(); i++) { 
        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
        boundRect[i] = boundingRect(Mat(contours_poly[i]));
    }
    
    Rect biggestRect = getBiggestRectBetween(boundRect[0], boundRect[1]);
    float heightRatio = getHeightRatioBetween(biggestRect, srcHSV);
    if (heightRatio > 0.85f) {
        return true;
    }
    
    return false;
}

Mat SudocubeExtractor::segmentGreenFrame(const Mat &hsvPicture) {
    Mat greenFrameSegmented;
    inRange(hsvPicture, Scalar(30, 150, 50), Scalar(95, 255, 255), greenFrameSegmented);
    
    return greenFrameSegmented;    
}

float SudocubeExtractor::getHeightRatioBetween(const Rect frameRect, const Mat &picture) {
    return (float) frameRect.size().height / (float) picture.size().height;
}

Rect SudocubeExtractor::getBiggestRectBetween(const Rect &rect1, const Rect &rect2) {
    if (rect1.area() > rect2.area()) {
        return rect1;
    }
    
    return rect2;
}

Rect SudocubeExtractor::isolateGreenFrame(const Mat &hsvPicture) {
    Mat segmentedGreenFrame = segmentGreenFrame(hsvPicture);
    
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(segmentedGreenFrame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    for(int i = 0; i < contours.size(); i++) { 
        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
        boundRect[i] = boundingRect(Mat(contours_poly[i]));
    }
    
    return getSmallestRectBetween(boundRect[0], boundRect[1]);
}

Rect SudocubeExtractor::getSmallestRectBetween(const Rect &rect1, const Rect &rect2) {
    if (rect1.area() < rect2.area()) {
        return rect1;
    }
    
    return rect2;
}

Mat cropSquare(const Mat &picture, const Rect &frameRect) {
    return picture(frameRect);
}

bool SudocubeExtractor::findAllSudokuSquares(const Mat &grayPicture, vector<vector<Point> > &squaresPoly, vector<Rect> &squaresRect) {
    // La valeur du threshold ne sera pas hardcodé(int 200) dans la version finale. Pour rendre l'algo
    // plus robuste on pourrais trouver la borne inférieur et supérieur des valeurs du threshold qui nous donnent
    // 47 carrés et au final faire (borneSup - borneInf) / 2 
    
    getSquares(grayPicture, 200, squaresPoly, squaresRect);
    removeInvalidSquares(squaresPoly, squaresRect);
    
    if (squaresRect.size() == 47) {
        return true;
    }
    
    return false;
}

void SudocubeExtractor::getSquares(const Mat &grayPicture, const int thresholdValue, vector<vector<Point> > &squaresPoly, vector<Rect> &squaresRect) {
    Mat segmentedSudoku;
    threshold(grayPicture, segmentedSudoku, thresholdValue, 150, THRESH_BINARY);
    
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(segmentedSudoku, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
   
    squaresPoly.resize(contours.size());
    squaresRect.resize(contours.size());
    
    for(int i = 0; i < contours.size(); i++) { 
        approxPolyDP(Mat(contours[i]), squaresPoly[i], 3, true);
        squaresRect[i] = boundingRect(Mat(squaresPoly[i]));
    }
}

void SudocubeExtractor::removeInvalidSquares(vector<vector<Point> > &squaresPoly, vector<Rect> &squaresRect) {
    vector<vector<Point> > validPoly;
    vector<Rect> validRect;
    
    for(int i = 0; i < squaresRect.size(); i++) {
        if(squaresRect[i].area() > 1500 && squaresRect[i].area() < 10000) {
            validRect.push_back(squaresRect[i]);
            validPoly.push_back(squaresPoly[i]);
        }
    }
    
    squaresPoly = validPoly;
    squaresRect = validRect;
}

void SudocubeExtractor::showSudokuSquares(Size pictureSize, const vector<vector<Point> > &squaresPoly, const vector<Rect> &squaresRect) {
    Mat drawing = Mat::zeros(pictureSize, CV_8UC3);

    //drawAllRect(drawing, squaresRect);
    drawAllPolygon(drawing, squaresRect, squaresPoly);
    
    showWindowWith("Squares found", drawing);
}

void SudocubeExtractor::drawAllPolygon(Mat &drawing, const vector<Rect> &squaresRect, const vector<vector<Point> > squaresPoly) {
    for(int i = 0; i < squaresRect.size(); i++) {
        drawContours(drawing, squaresPoly, i, white, 1, 8, vector<Vec4i>(), 0, Point());
    }
}

void SudocubeExtractor::drawAllRect(Mat &drawing, const vector<Rect> &squaresRect) {
    for(int i = 0; i < squaresRect.size(); i++) {
        rectangle(drawing, squaresRect[i].tl(), squaresRect[i].br(), white, 1, 4, 0);
    }
}

void SudocubeExtractor::showWindowWith(const char* name, const Mat &mat) {
    namedWindow(name, CV_WINDOW_AUTOSIZE);
    imshow(name, mat);
}

int main(int argc, char** argv) {
    SudocubeExtractor extractor;
    
    extractor.isSudokuFrameVisible();
    
    /*src = imread(argv[1], 1);
    cvtColor(src, srcGray, CV_BGR2GRAY);
    cvtColor(src, srcHSV, CV_BGR2HSV);

    showWindowWith("Original", src);

    if (isSudokuFrameVisible(srcHSV) == true) {
        cout << "The square is big enough and centered" << endl;

        Rect greenFrame = isolateGreenFrame(srcHSV);
        showWindowWith("Green frame isolated", cropSquare(src, greenFrame));
        cout << "The green frame as been isolated" << endl;
        
        vector<vector<Point> > squaresPoly;
        vector<Rect> squaresRect;
        if (findAllSudokuSquares(cropSquare(srcGray, greenFrame), squaresPoly, squaresRect)) {
            showSudokuSquares(greenFrame.size(), squaresPoly, squaresRect);
            cout << "All the squares have been found" << endl;
            
            //Ici, on peux trouver le carré rouge en faisant une segmentation par couleur.
            //C'est à dire comme j'ai fait pour le cadre vert
        } else {
            cout << "Didn't find all the squares" << endl;
        }
    }

    waitKey(0);*/
   
    
    return(0);
}
