/*
 * mainTest.cpp
 *
 *  Created on: Mar 30, 2013
 *      Author: Diane Fournier
 */

#include "localisation.h"
#include <cctype>
#include <stdio.h>
#include <string.h>

using namespace std;
using namespace cv;

int main( int argc, char** argv )
{
	string inputImage;
	int orientation, locateOK, findAngleOK;
	Mat image, intrinsic, extrinsic, distortionMatrix;
	double paramX, paramY;
	int i;
//	for(i = 0; i < argc; i++)
//	{
//		const char* s = argv[i];
//		if(strcmp(s, "-o") == 0)
//		{
//			if(sscanf(argv[++i]), "%d", &orientation) < 0 || orientation > 3)
//			    return fprintf( stderr, "Invalid orientation\n" ), -1;
//		}
//		else if(strcmp(s, "-i") == 0)
//			    inputImage = argv[++i];
//		}
//		else if (strcmp(s, "-p") == 0)
//
//			// TODO : read parameter file
//		}
//		else
//		{
//			return fprintf( stderr, "Unknown option %s", s ), -1;
//		}
//	}
	image = imread(inputImage,1);
	localisation::localisation locate(image, orientation);
	locate.initLocalisation(intrinsic, distortionMatrix, extrinsic, paramX, paramY);
}






