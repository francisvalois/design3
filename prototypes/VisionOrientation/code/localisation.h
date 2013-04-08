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

// Changer selon la resolution des images capturees

#define PIXEL_X 640
#define PIXEL_Y 480

#define NORTH 0
#define SOUTH 1
#define EAST 2
#define WEST 3

#define NE_CORNER 0
#define NW_CORNER 1
#define SE_CORNER 2
#define SW_CORNER 3

#define L_NE_CORNER 4
#define R_NE_CORNER 5
#define L_NW_CORNER 6
#define R_NW_CORNER 7

#define L_SE_CORNER 8
#define R_SE_CORNER 9
#define L_SW_CORNER 10
#define R_SW_CORNER 11

#define SW_INS_CORNER 12
#define SE_INS_CORNER 13
#define NW_INS_CORNER 14
#define NE_INS_CORNER 15


class localisation
{
	class KnownPoint{
		public:
			double x;
			double y;
			int ID;
		};
public:
	localisation();

	virtual ~localisation();

	//   Initialisation
	//   -Initialisation des parametres a partir d'une matrice de transformation
	//   -correction de la distortion
	//   -transformation de l'image en mode HSV
	void initLocalisation(cv::Mat & image, int orientation, std::string & params);

	//   Determine l'angle par rapport au nord de la table
	//   La valeur est retournee dans la reference
	//   La fonction retourne 0 si succes
	void findAngle(double & angle);

	//   Determine l'angle en radians par rapport a la normale du mur d'en face
	void angleRelativeToWall(std::vector<cv::Vec2f> wallLines, double & wallAngle);

	//   Position dans la representation de la table
	//   L'image doit montrer au moins un repere:
	//   - coin de couleur
	//   - ligne rouge
	//   - carre vert pour le dessin
	int findPosition(double coordonnees[2]);

	void getTransfoMatrix(std::vector<double> & matrix);

//private:

	int coinOrange(std::vector<cv::Vec2f> orangeLines);

	int coinBleu(std::vector<cv::Vec2f> orangeLines);

	int ligneVerte(std::vector<cv::Vec2f> greenLines);

	int ligneRouge(std::vector<cv::Vec2f> redLines);

	int findWallLines(std::vector<cv::Vec2f> & wallLines);

	int findPoints(std::vector<KnownPoint> points);

	void findIntersection(cv::Vec2f & ligne1, cv::Vec2f & ligne2, cv::Point & pt);

	void positionRelativeToTarget(KnownPoint & imagePoint, KnownPoint & objectPoint);

	void positionRelativeToRobot(KnownPoint & point);

	void translateToTableReference(KnownPoint & P1R, KnownPoint & P2R, cv::Point & t);

	void translateToTableReference(KnownPoint & P1R, double & theta, cv::Point & t);

private:
	// image en entree
	cv::Mat mImage;
	// parametres pour passer de l'image au repere de calibrage
	double m11, m12, m13, m14, m21, m22, m23, m24, m31, m32, m33, m34;
	// matrice des parametres de distortion
	cv::Mat mDistortionMatrix;
	// orientation par rapport a la table
	int mState;
	// parametres pour transformation du repere cible au repere robot
	std::vector<double> mParams;
	// liste des points de repere avec leurs coordonnees
	std::vector<KnownPoint> reperes;
	// indicateur d'etat d'initialisation
	bool initOK;
};

#endif /* LOCALISATION_H_ */
