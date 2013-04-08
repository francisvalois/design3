/*
 * capture.cpp
 *
 *  Created on: Mar 23, 2013
 *      Author: diane
 */
 #include "cv.h"
 #include "highgui.h"
 #include <stdio.h>
using namespace cv;

 // A Simple Camera Capture Framework
 int main() {

   system("./configure_camera.sh");
   VideoCapture cap(0);
   //cap.set(CV_CAP_PROP_FRAME_WIDTH, 1600);
   //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1200);
   //cap.set(CV_CAP_PROP_BRIGHTNESS, 0.509803);
   //cap.set(CV_CAP_PROP_BRIGHTNESS, 0.70);
   //cap.set(CV_CAP_PROP_CONTRAST, 0.196078);
   //cap.set(CV_CAP_PROP_CONTRAST, 0.25);
   //cap.set(CV_CAP_PROP_SATURATION, 0.176470);
   //cap.set(CV_CAP_PROP_SATURATION, 0.30);
   //cap.set(CV_CAP_PROP_GAIN, 1);

   int value;

   cvNamedWindow("mywindow", CV_WINDOW_KEEPRATIO);
   createTrackbar("trackbar","mywindow", &value, 100);

   Mat picture;
   //cap >> picture;
   //CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
   //if ( !capture ) {
   //  fprintf( stderr, "ERROR: capture is NULL \n" );
   //  getchar();
   //  return -1;
   //}
   // Create a window in which the captured images will be presented
   //cvNamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );
   // Show the image captured from the camera in the window and repeat

   int waitForPicture = -1;
   while ( waitForPicture == -1 ) {
     // Get one frame
	   cap >> picture;
	   imshow("mywindow", picture);
     //IplImage* frame = cvQueryFrame( capture );
	   waitForPicture = waitKey(50);
     //if ( !frame ) {
     //  fprintf( stderr, "ERROR: frame is null...\n" );
     //  getchar();
     //  break;
     //}
     //cvShowImage( "mywindow", picture );
     // Do not release the frame!
     //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
     //remove higher bits using AND operator
     //if ( (cvWaitKey(10) & 255) == 27 ) break;
   }
   // Release the capture device housekeeping
   //cvReleaseCapture( &capture );
   //cvDestroyWindow( "mywindow" );
   vector<int> compression_params;
   compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
   compression_params.push_back(9);
   string filename = "test.png";
   //Mat pict = picture.clone();

   imwrite(filename, picture, compression_params);
   return 0;
 }



