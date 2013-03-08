#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

//const int TABLEX = 232;
//const int TABLEY = 115;

Scalar white = Scalar(255, 255, 255);
Scalar blue = Scalar(255, 0, 0);
Scalar black = Scalar(0, 0, 0);


void showWorkspace(int [TABLE_X+1][TABLE_Y+1]);
void showWindowWith(const char*, const Mat &);
void colorPixel(Mat&, Scalar, int, int);

void showWorkspace(int table[TABLE_X+1][TABLE_Y+1]) {
    Mat workspace = Mat(TABLE_X+1, TABLE_Y+1, CV_8UC3, white);
    
    for (int y = 0; y < (TABLE_Y+1); y++) {
    	for (int x = 0; x < (TABLE_X+1); x++) {
            if (table[x][y] == 1) {
                colorPixel(workspace, black, x, y);
            }

        }
    } 
    transpose(workspace, workspace);

    showWindowWith("Workspace", workspace);
}

void colorPixel(Mat &mat, Scalar color, int x, int y) {
    mat.at<cv::Vec3b>(x,y)[0] = color[0];
    mat.at<cv::Vec3b>(x,y)[1] = color[1];
    mat.at<cv::Vec3b>(x,y)[2] = color[2];
}

void showWindowWith(const char* name, const Mat &mat) {
    namedWindow(name, CV_WINDOW_AUTOSIZE);
    imshow(name, mat);
    waitKey(0);
}

