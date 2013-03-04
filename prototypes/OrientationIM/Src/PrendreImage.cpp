
//#include "cv.h"
//#include "highgui.h"
//#include "string.h"
//#include <sstream>
//#include <string>
//
//using namespace cv;
//
//string i2string(int i) {
// std::ostringstream buffer;
// buffer << i;
// return buffer.str();
//}
//
//int paramater;
//string VideoCaptureView= "videoCapture";
//string ImagecaptureView= "ImageCapture";
//string ExtensionString = ".tif";
//string ImageName= "ImagesCapture2/Image";
//
//VideoCapture cap(1);
//Mat image;
//
//
//
//int count=1;
//
//int main(int, char**)
//{
//
////	namedWindow(VideoCaptureView,CV_WINDOW_AUTOSIZE);
////	namedWindow(ImagecaptureView,CV_WINDOW_AUTOSIZE);
////
////    if(!cap.isOpened())  // check if we succeeded
////        return -1;
////
////    for(;;)
////    {
////    	cap >> image;
////        if (image.data){
////        	imshow(VideoCaptureView, image);
////        }
////        int key = waitKey(30);
////
////    	if(key == ' ') {
////    		string countStr = i2string(count);
////    		imwrite(ImageName+countStr+ExtensionString,image);
////    		imshow(ImagecaptureView, image);
////    		count++;
////    	}else if(key == 'q') {
////    		break;
////    	}
////    }
////
////    return 0;
//}
//
