#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

using namespace cv;
using namespace std;


int counter = 0;

void saveImage(Mat&, string&);


int main(int, char**)
{
    VideoCapture cap(0);
    if(!cap.isOpened())
        return -1;

    namedWindow("pic",1);
    for(;;)
    {
        Mat frame;
        cap >> frame;
        imshow("pic", frame);

		int keyPressed = waitKey(30);
		if (keyPressed == 13) {
			counter++;
			std::string result;
			std::stringstream sstm;
			sstm << "no" << counter << "pict.jpeg";
			result = sstm.str();
			saveImage(frame, result);
		} else if (keyPressed != 13 && keyPressed >= 0) {
			break;
		}
			
    }

    return 0;
}

void saveImage(Mat &pict, string &filename) {
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(100);

    imwrite(filename, pict, compression_params);

}

