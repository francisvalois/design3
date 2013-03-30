/*
 * localisation.h
 *
 *  Created on: Mar 26, 2013
 *      Author: Diane Fournier
 */

#ifndef LOCALISATION_H_
#define LOCALISATION_H_

#include <cv.h>
#include "core/core.hpp"
#include "imgproc/imgproc.hpp"
#include "calib3d/calib3d.hpp"
#include "highgui/highgui.hpp"

enum { NORTH = 0, SOUTH = 1, EAST = 2, WEST = 3 };

class localisation
{
public:
	localisation(cv::Mat &image, char &orientation);

	virtual ~localisation();

	//   Determine l'angle par rapport au nord de la table
	//   La valeur est retournee dans la reference
	//   La fonction retourne 0 si succes
	int findAngle(double &angle);

	//   Determine l'angle par rapport a la normale du mur d'en face
	int angleRelativeToWall(double &wallAngle);

	//   Position dans la representation de la table
	//   L'image doit montrer au moins un repere:
	//   - coin de couleur
	//   - ligne rouge
	//   - carre vert pour le dessin
	int findPosition(double position[2]);
private:

	//   Initialisation
	//   -Initialisation des parametres a partir d'une matrice de transformation
	//   -correction de la distortion
	//   -transformation de l'image en mode HSV
	//   Retourne 0 si succes
	int initLocalisation();

	int coinOrange(double imPos[2]);

	int coinBleu(double imPos[2]);

	void findWallLines(double coordLine[2]);

	void positionRelativeToTarget(double imagePoint[2], double objectPoint[2]);

	void positionRelativeToRobot(double point[2]);

	cv::Mat mImage;
	double m11, m12, m13, m14, m21, m22, m23, m24, m31, m32, m33, m34;
	cv::Mat mDistortionMatrix;
	int mState;
};

#endif /* LOCALISATION_H_ */
