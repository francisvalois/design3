

#include "Calib.h"

Calib::Calib(double* U, double* V) {
	m_U = *U;
	m_V = *V;
	m_X = 0;
	m_Y = 0;
	m_Z = 0;

	zhang = new Zhang();
	noFile = false;
}

Calib::~Calib() {
	// TODO Auto-generated destructor stub
}

void Calib::OnLoadParameters(const string* inFichierParams) {
	if (zhang->SetParameters(inFichierParams))
		noFile = true;
	else
		noFile = false;
}

void Calib::OnApplyZhang(double* ioX, double* ioY,double* ioZ) {

	//	ofstream errorFile("ErrorFile.log");
	//	cerr.rdbuf(errorFile.rdbuf());
	if (!noFile) {
		zhang->Apply(m_U, m_V, m_X, m_Y, m_Z);
	}

	*ioX = m_X;
	*ioY = m_Y;
	*ioZ = m_Z;

}
