/*
 * testOpenCV.cpp
 *
 *  Created on: Mar 20, 2013
 *      Author: diane
 */
#include <cv.h>
#include <highgui.h>
#include "imgproc/imgproc.hpp"

using namespace cv;

int main( int argc, char** argv )
{
  Mat image, gray;
  Mat undistorted;
  double d[] = {5.3049382516541385e-02, -8.3096662051120498e-02,-1.1345776472333211e-03, 2.5208106546648732e-03,-1.2073151061566005e-01};
  double m[3][3] = {{1.3304479850993566e+03,0.,8.1925129148813471e+02},{0.,1.3290483123549550e+03,5.5096732315473776e+02},{0., 0., 1.}};
  Mat intrinsic = Mat(3,3,CV_64F, m);
  Mat distMat = Mat(1,5, CV_64F, d);
  string file = "/home/diane/workspace/testOpenCV/Debug/image13.png";
  image = imread( file, 1 );

  //undistort(image, undistorted, intrinsic, distMat);

  if( argc != 1 || !image.data )
    {
      printf( "No image data \n" );
      return -1;
    }

  vector<Point2f> pointbuf;
  bool found;
  cvtColor(image, gray, CV_BGR2GRAY);
  Size boardSize;
  boardSize.width = 9;
  boardSize.height = 6;
  found = findChessboardCorners( image, boardSize, pointbuf,
                      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
  if(found) cornerSubPix( gray, pointbuf, Size(11,11),
              Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
  if(found)
              drawChessboardCorners( image, boardSize, Mat(pointbuf), found );
  vector<Point3f> objectPoints(1);
  objectPoints.resize(0);
  int i,j;
  for(i = 0; i < boardSize.height; i++ )
      for(j = 0; j < boardSize.width; j++ )
             objectPoints.push_back(Point3f(float(j*22),float(i*22), 0));
  int sizeAvant = objectPoints.size();

  objectPoints.resize(pointbuf.size(),objectPoints[0]);
  int sizeApres = objectPoints.size();
  int sizeBuf = pointbuf.size();
  Mat rvec,tvec;
  bool ok;
  ok = solvePnP(objectPoints,pointbuf,intrinsic,distMat,rvec,tvec);
  Mat rotMatrix;
  Rodrigues(rvec,rotMatrix);
  Mat extrinsic;
  //vector<vector<float> > ext;
  hconcat(rotMatrix,tvec,extrinsic);

  //for(i=0;i<4;i++)
  //{
  //	  for(j=0;j<3;j++)
  //	  {
  //		  Mat ligne;
  //		  if(i < 3)
  //			  ligne.push_back(rotMatrix(i,j));
  //	  }
  //}
  string filename = "paramsExt";
  FileStorage fs( filename, FileStorage::WRITE );
  fs << "rotation_vector" << rvec;
  fs << "translation_vector" << tvec;
  fs << "rotation matrix" << rotMatrix;
  fs << "extrinsic" << extrinsic;


  //namedWindow( "Display Image", CV_WINDOW_AUTOSIZE );
  //namedWindow("Display undistorted image", CV_WINDOW_AUTOSIZE);
  //imshow( "Display Image", image);
  //imshow("Display undistorted image", CV_WINDOW_AUTOSIZE);
  //vector<int> compression_params;
  //compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  //compression_params.push_back(9);
  //imwrite("undistImage.png", undistorted, compression_params);

  waitKey(0);

  return 0;
}




