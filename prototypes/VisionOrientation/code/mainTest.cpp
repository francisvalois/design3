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
	string inputImage = "/home/diane/workspace/localisation/Debug/1717_568_210.png";
	int orientation, locateOK, findAngleOK;
	orientation = SOUTH;
	Mat image;
	string filename = "/home/diane/workspace/localisation/Debug/paramsExt2.xml";


	image = imread(inputImage,1);
	Localisation locate;
	locate.initLocalisation(image, orientation, filename);

	int nbLignesMur;
	vector<Vec2f> lignesMur;
	nbLignesMur = locate.findWallLines(lignesMur);
	cout << nbLignesMur << endl;
	double angle;
	locate.angleRelativeToWall(lignesMur, angle);
	double degres = 360*angle/(2*CV_PI);
	cout << degres << endl;
	double globalAngle, globDegres;
	locate.findAngle(globalAngle);
	globDegres = 360*globalAngle/(2*CV_PI);
	cout << globDegres << endl;

	//vector<Localisation::KnownPoint> pointsID;
	//Localisation::KnownPoint pointRobot;
	//locate.findPoints(pointsID);
	//cout << pointsID.size() << endl;


	//locate.positionRelativeToTarget(pointsID[0], pointRobot);
	//locate.positionRelativeToRobot(pointRobot);

	//pointRobot.ID = NW_CORNER;
	//cout << "Coordonnee du coin : " << pointRobot.x << ", " << pointRobot.y << endl;
	//Point t;
	//double angle1 = (270.0/360)*2*CV_PI;
	//cout << angle1 << endl;
	//locate.translateToTableReference(pointRobot, angle1, t);
	//cout << "Coordonnees robot : " << t.x << ", " << t.y << endl;
}






