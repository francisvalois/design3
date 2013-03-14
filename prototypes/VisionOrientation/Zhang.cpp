#include "Zhang.h"
#include <iostream>


string CalibParamsFileNames[10];

Zhang::Zhang() {
	camera = new CameraModel();
	ErrorIDZHANG=-1;

}

Zhang::~Zhang() {
	delete camera;
}

int Zhang::SetParameters(int inIndexFile) {

	vector<double> lBuffer;
string fileName = CalibParamsFileNames[inIndexFile-1];

	ifstream fichier(fileName.c_str(), ios::in);
	if (!fichier.fail()) {
		double ligne;
		while (fichier >> ligne) {
			lBuffer.push_back(ligne);
		}

		this->ErrorIDZHANG = FICHIER_PARAMS_LU;

		fichier.close();

	} else {
		this->ErrorIDZHANG = FICHIER_PARAMS_NON_LU;
		cerr << "ErrorID ZHANG:" << this->ErrorIDZHANG << endl;
	}

	double IntrinsicDataBufL[7];
	double ExtrinsicDataBufL[12];

	if (this->ErrorIDZHANG == FICHIER_PARAMS_LU) {

		for (int i = 0; i < 7; i++) {
			IntrinsicDataBufL[i] = lBuffer[i];
		}
		for (int i = 0; i < 12; i++) {
			ExtrinsicDataBufL[i] = lBuffer[i + 7];
		}

		camera->TheIntrinsicParam.SetAlphaU(IntrinsicDataBufL[0]);
		camera->TheIntrinsicParam.SetGamma(IntrinsicDataBufL[1]);
		camera->TheIntrinsicParam.SetAlphaV(IntrinsicDataBufL[2]);
		camera->TheIntrinsicParam.SetU0(IntrinsicDataBufL[3]);
		camera->TheIntrinsicParam.SetV0(IntrinsicDataBufL[4]);
		camera->TheIntrinsicParam.SetK1(IntrinsicDataBufL[5]);
		camera->TheIntrinsicParam.SetK2(IntrinsicDataBufL[6]);

		camera->TheExtrinsicParam.SetRotationMatrix(ExtrinsicDataBufL[0],
				ExtrinsicDataBufL[1], ExtrinsicDataBufL[2],
				ExtrinsicDataBufL[3], ExtrinsicDataBufL[4],
				ExtrinsicDataBufL[5], ExtrinsicDataBufL[6],
				ExtrinsicDataBufL[7], ExtrinsicDataBufL[8]);

		camera->TheExtrinsicParam.SetTranlation(ExtrinsicDataBufL[9],
				ExtrinsicDataBufL[10], ExtrinsicDataBufL[11]);
	}else
		return -1;

	return 0;
}

int Zhang::Apply(double u, double v, double &x, double &y, double &z) {

	camera->PixelDist2ImgPlaneUndist(u, v, x, y, z);

	Matrix intrinsic;
	Matrix extrinsic;
	Matrix r3;
	Matrix Tcw;
	Matrix V;
	Matrix p1, p2;
	Matrix Pw, Pc;

	intrinsic.SetSize(3, 3);
	intrinsic.I();
	intrinsic = camera->TheIntrinsicParam.GetIntrinsicParamMatrix();

	extrinsic.SetSize(4, 4);
	extrinsic.I();
	extrinsic = camera->TheExtrinsicParam.GetExtrinsicParamMatrix();

	r3.SetSize(3, 1);
	r3.I();
	r3(0, 0) = extrinsic(0, 2);
	r3(1, 0) = extrinsic(1, 2);
	r3(2, 0) = extrinsic(2, 2);

	Tcw.SetSize(3, 1);
	Tcw.I();
	Tcw(0, 0) = extrinsic(0, 3);
	Tcw(1, 0) = extrinsic(1, 3);
	Tcw(2, 0) = extrinsic(2, 3);

	V.SetSize(3, 1);
	V.I();
	V(0, 0) = x;
	V(1, 0) = y;
	V(2, 0) = z;

	p1.SetSize(1, 1);
	p1.I();

	p2.SetSize(1, 1);
	p2.I();

	r3.Transpose();
	p1 = r3 * Tcw;
	p2 = r3 * V;

	if ((p2(0, 0) > 0.0 ? p2(0, 0) : -p2(0, 0)) < 2.2e-16) {
		x = -1;
		y = -1;
		z = -1;
		return -1;
	}

	double p3 = p1(0, 0) / p2(0, 0);

	Pc.SetSize(4, 1);
	Pc.I();

	Pc(0, 0) = V(0, 0) * p3;
	Pc(1, 0) = V(1, 0) * p3;
	Pc(2, 0) = V(2, 0) * p3;
	Pc(3, 0) = 1;

	Pw.SetSize(4, 1);
	Pw.I();

	extrinsic.Invert();
	Pw = extrinsic * Pc;

	x = Pw(0, 0);
	y = Pw(1, 0);
	z = Pw(2, 0);

	return 0;
}