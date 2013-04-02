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
enum {NE_CORNER = 0, NW_CORNER = 1, SE_CORNER = 2, SW_CORNER = 3};
enum {L_NE_CORNER = 4, R_NE_CORNER = 5, L_NW_CORNER = 6, R_NW_CORNER = 7};
enum {L_SE_CORNER = 8, R_SE_CORNER = 9, L_SW_CORNER = 10, R_SW_CORNER = 11};
enum {S_RED_LINE_W = 12, N_RED_LINE_W = 13, S_RED_LINE_E = 14, N_RED_LINE_E =15};
enum {W_INS_CORNER_S = 16, E_INS_CORNER_S =17, W_INS_CORNER_N =18, E_INS_CORNER_N = 19};


class localisation
{
	class KnownPoint{
		public:
			double x;
			double y;
			int ID;
		};
public:
	localisation(cv::Mat &image, int orientation);

	virtual ~localisation();

	//   Initialisation
	//   -Initialisation des parametres a partir d'une matrice de transformation
	//   -correction de la distortion
	//   -transformation de l'image en mode HSV
	//   Retourne 0 si succes
	int initLocalisation(cv::Mat & intrinsic, cv::Mat & distorsionMatrix, cv::Mat & extrinsic, double paramX, double paramY);

	//   Determine l'angle par rapport au nord de la table
	//   La valeur est retournee dans la reference
	//   La fonction retourne 0 si succes
	int findAngle(double &angle);

	//   Determine l'angle en radians par rapport a la normale du mur d'en face
	int angleRelativeToWall(double &wallAngle);

	//   Position dans la representation de la table
	//   L'image doit montrer au moins un repere:
	//   - coin de couleur
	//   - ligne rouge
	//   - carre vert pour le dessin
	int findPosition(double coordonnees[2]);

private:

	int coinOrange(std::vector<cv::Vec2f> orangeLines);

	int coinBleu(std::vector<cv::Vec2f> orangeLines);

	int ligneVerte(std::vector<cv::Vec2f> greenLines);

	int ligneRouge(std::vector<cv::Vec2f> redLines);

	int findWallLines(std::vector<cv::Vec2f> wallLines);

	int findPoints(std::vector<KnownPoint> points);

	void findIntersection(cv::Vec2f & ligne1, cv::Vec2f & ligne2, cv::Point & pt);

	void positionRelativeToTarget(KnownPoint imagePoint, KnownPoint objectPoint);

	void positionRelativeToRobot(KnownPoint point);

	int translateToTableReference(KnownPoint P1A, KnownPoint P2A, KnownPoint P1R, KnownPoint P2R);

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
