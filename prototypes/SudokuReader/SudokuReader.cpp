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

Scalar white = Scalar(255, 255, 255);

bool isSudokuFrameVisible(const Mat &hsvPicture);
Mat segmentGreenFrame(const Mat&);
Rect getBiggestRectBetween(const Rect &, const Rect &);
float getHeightRatioBetween(const Rect, const Mat &);

Rect isolateGreenFrame(const Mat &);
Rect getSmallestRectBetween(const Rect &, const Rect &);
Mat cropSquare(const Mat &, const Rect &) ;

bool findAllSudokuSquares(const Mat &, vector<vector<Point> > &, vector<Rect> &);
void getSquares(const Mat &, const int, vector<vector<Point> > &, vector<Rect> &);
void removeInvalidSquares(vector<vector<Point> > &, vector<Rect> &);

void showSudokuSquares(Size, const vector<vector<Point> > &, const vector<Rect> &);
void drawAllRect(Mat &, const vector<Rect> &);
void drawAllPolygon(Mat &, const vector<Rect> &, const vector<vector <Point> >);

void showWindowWith(const char*, const Mat &);


int main(int argc, char** argv) {
    src = imread(argv[1], 1);
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

Rect isolateGreenFrame(const Mat &hsvPicture) {
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
    return getSmallestRectBetween(boundRect[0], boundRect[1]);
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


/**
* Isole tous les carrés du jeu de sudoku. Retourne une liste de polygone et une liste de Rect pour
* traiter ulterieurement les carrés 
*
* @return bool Vrai si l'algorithme trouve tous les carrés du jeu de sudoku (47 au total).
*/
bool findAllSudokuSquares(const Mat &grayPicture, vector<vector<Point> > &squaresPoly, vector<Rect> &squaresRect) {
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

void getSquares(const Mat &grayPicture, const int thresholdValue, vector<vector<Point> > &squaresPoly, vector<Rect> &squaresRect) {
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

void removeInvalidSquares(vector<vector<Point> > &squaresPoly, vector<Rect> &squaresRect) {
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

void showSudokuSquares(Size pictureSize, const vector<vector<Point> > &squaresPoly, const vector<Rect> &squaresRect) {
    Mat drawing = Mat::zeros(pictureSize, CV_8UC3);

    //drawAllRect(drawing, squaresRect);
    drawAllPolygon(drawing, squaresRect, squaresPoly);
    
    showWindowWith("Squares found", drawing);
}

void drawAllPolygon(Mat &drawing, const vector<Rect> &squaresRect, const vector<vector<Point> > squaresPoly) {
    for(int i = 0; i < squaresRect.size(); i++) {
        drawContours(drawing, squaresPoly, i, white, 1, 8, vector<Vec4i>(), 0, Point());
    }
}

void drawAllRect(Mat &drawing, const vector<Rect> &squaresRect) {
    for(int i = 0; i < squaresRect.size(); i++) {
        rectangle(drawing, squaresRect[i].tl(), squaresRect[i].br(), white, 1, 4, 0);
    }
}

void showWindowWith(const char* name, const Mat &mat) {
    namedWindow(name, CV_WINDOW_AUTOSIZE);
    imshow(name, mat);
}
