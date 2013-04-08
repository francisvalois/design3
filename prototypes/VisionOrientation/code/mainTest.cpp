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
	string inputImage = "/home/diane/workspace/localisation/Debug/244_166.4_66.2.png";
	int orientation, locateOK, findAngleOK;
	orientation = WEST;
	Mat image;
	string filename = "/home/diane/workspace/localisation/Debug/paramsExt2.xml";


	image = imread(inputImage,1);
	localisation locate;
	locate.initLocalisation(image, orientation, filename);
	vector<double> test;
	locate.getTransfoMatrix(test);
	cout << test[0] << endl;

	int nbLignesMur;
	vector<Vec2f> lignesMur;
	nbLignesMur = locate.findWallLines(lignesMur);
	cout << nbLignesMur << endl;
	cout << lignesMur.size() << endl;
	double angle;
	locate.angleRelativeToWall(lignesMur, angle);
	double degres = 360*angle/(2*CV_PI);
	cout << degres << endl;
	double globalAngle;
	locate.findAngle(globalAngle);
	degres = 360*globalAngle/(2*CV_PI);
	cout << degres << endl;
}






