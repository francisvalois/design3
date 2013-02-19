
#ifndef CALIB_H_
#define CALIB_H_


#include <iostream>
#include "Zhang/Zhang.h"
#include <string>

class Calib {
public:

	Calib( double* U,  double* V); // standard constructor

	virtual ~Calib();

	Zhang* zhang;

	// Dialog Data
	double m_U;
	double m_V;
	double m_X;
	double m_Y;
	double m_Z;
	void OnApplyZhang(double* ioX, double* ioY,double* ioZ);
	void OnLoadParameters(const string* inFichierParams);

private:

	bool noFile;

};

#endif /* CALIB_H_ */
