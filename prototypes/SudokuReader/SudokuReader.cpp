#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

Mat src; 
Mat srcGray;
Mat srcHSV;

int thresh_cases = 150;
int thresh_contour = 150;
int max_thresh = 255;

Scalar white = Scalar(255, 255, 255);

bool isSudokuFrameVisible(const Mat &hsvPicture);
Mat segmentGreenFrame(const Mat&);
Rect getBiggestRectBetween(const Rect &, const Rect &);
float getHeightRatioBetween(const Rect, const Mat &);

Mat isolateGreenFrame(const Mat &, const Mat &);
Rect getSmallestRectBetween(const Rect &, const Rect &);
Mat cropSquare(const Mat &, const Rect &) ;

void thresh_cases_callback(int, void*);
void drawAllSquare(Mat &, const vector<Rect> &);
void drawAllPolygon(Mat &, const vector<Rect> &, const vector<vector <Point> >);

void showWindowWith(const char*, const Mat &);


int main(int argc, char** argv) {
    src = imread(argv[1], 1);
    cvtColor(src, srcGray, CV_BGR2GRAY);
    cvtColor(src, srcHSV, CV_BGR2HSV);

    //Affichage de l'image originale
    showWindowWith("Original", src);
  
    //Methode pour verifier si le sudoku est dans limage et
    if (isSudokuFrameVisible(srcHSV) == true) {
        cout << "The square is big enough and centered" << endl;
        
        //Isolation du cadre vert du sudoku
        Mat greenFrame = isolateGreenFrame(src, srcHSV);
        showWindowWith("Cadre vert isole", greenFrame);
    }

    waitKey(0);
    return(0);
}


/**
* Sert a verifier si le carre du sudoku 3d est suffisament visible dans lecran.
* Grosso modo on verifie seulement si le cadre vert fait faut moin 
*
* @param hsvPicture Une image convertie en couleur HSV
*/
bool isSudokuFrameVisible(const Mat &hsvPicture) {
    Mat greenFrameSegmented = segmentGreenFrame(hsvPicture);
    
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(greenFrameSegmented, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    
    //Si aucun carre a ete trouve ou trop de carres(on ne peux identifier correctement les carres si plus de 2)
    if (contours.size() != 2) {
        return false;
    } 
    
    //Recuperation des polygones des carres et des boundingBox
    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    for(int i = 0; i < contours.size(); i++) { 
        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
        boundRect[i] = boundingRect(Mat(contours_poly[i]));
    }
    
    //Le ratio de la hauteur entre limage et le cadre vert doit etre le plus proche de 1
    //sinon le sudoku ne peux etre lu
    Rect biggestRect = getBiggestRectBetween(boundRect[0], boundRect[1]);
    float heightRatio = getHeightRatioBetween(biggestRect, hsvPicture);
    if (heightRatio > 0.85f) {
        return true;
    }
    
    return false;
}

Mat segmentGreenFrame(const Mat &hsvPicture) {
    Mat greenFrameSegmented;
    inRange(hsvPicture, Scalar(30, 150, 50), Scalar(95, 255, 255), greenFrameSegmented);
    
    return greenFrameSegmented;    
}

float getHeightRatioBetween(const Rect frameRect, const Mat &picture) {
    return (float) frameRect.size().height / (float) picture.size().height;
}

Rect getBiggestRectBetween(const Rect &rect1, const Rect &rect2) {
    if (rect1.area() > rect2.area()) {
        return rect1;
    }
    
    return rect2;
}

Mat isolateGreenFrame(const Mat &picture, const Mat &hsvPicture) {
    Mat segmentedGreenFrame = segmentGreenFrame(hsvPicture);
    
    //Recuperation des contours
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(segmentedGreenFrame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    //Recuperation des polygones et des Rect
    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    for(int i = 0; i < contours.size(); i++) { 
        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
        boundRect[i] = boundingRect(Mat(contours_poly[i]));
    }
    
    //On ne prend que linterieur du cadre vert
    Rect smallestRect = getSmallestRectBetween(boundRect[0], boundRect[1]);
    
    return cropSquare(picture, smallestRect);
}

Rect getSmallestRectBetween(const Rect &rect1, const Rect &rect2) {
    if (rect1.area() < rect2.area()) {
        return rect1;
    }
    
    return rect2;
}

Mat cropSquare(const Mat &picture, const Rect &frameRect) {
    return picture(frameRect);
}

void thresh_cases_callback(int, void*) {
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    threshold( srcGray, threshold_output, thresh_cases, 150, THRESH_BINARY );

    findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());

    for(int i = 0; i < contours.size(); i++) { 
        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
        boundRect[i] = boundingRect(Mat(contours_poly[i]));
    }

    Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);

    drawAllSquare(drawing, boundRect);
    drawAllPolygon(drawing, boundRect, contours_poly);
    
    showWindowWith("Squares", drawing);
}

void drawAllPolygon(Mat &drawing, const vector<Rect> &boundRect, const vector<vector<Point> > contours_poly) {
    for(int i = 0; i < boundRect.size(); i++) {
        if(boundRect[i].area() > 1500 && boundRect[i].area() < 10000) {
            drawContours(drawing, contours_poly, i, white, 1, 8, vector<Vec4i>(), 0, Point());
        }
    }
}

void drawAllSquare(Mat &drawing, const vector<Rect> &boundRect) {
    for(int i = 0; i < boundRect.size(); i++) {
        if(boundRect[i].area() > 1500 && boundRect[i].area() < 10000) {
            rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), white, 1, 4, 0);
        }
    }
}

void showWindowWith(const char* name, const Mat &mat) {
    namedWindow(name, CV_WINDOW_AUTOSIZE);
    imshow(name, mat);
}


/*
// Methodes pour dessiner les contours et les Rect... utilise pour debu
for(int i = 0; i < boundRect.size(); i++) {
    drawContours(drawing, contours_poly, i, white, 1, 8, vector<Vec4i>(), 0, Point());
}
Mat drawing = Mat::zeros(segmentedGreenFrame.size(), CV_8UC3);
for(int i = 0; i < boundRect.size(); i++) {
    rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), white, 1, 4, 0);
}
*/
