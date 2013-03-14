#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include <string>
#include <sstream>
#include <iostream>

static std::string CalibParamsFileNames[11] = {
		"/home/NetBeansProjects/integration/ParamsFile/Params1.txt",
		"/home/NetBeansProjects/integration/ParamsFile/Params2.txt",
		"/home/NetBeansProjects/integration/ParamsFile/Params3.txt",
		"/home/NetBeansProjects/integration/ParamsFile/Params4.txt",
		"/home/NetBeansProjects/integration/ParamsFile/Params5.txt",
		"/home/NetBeansProjects/integration/ParamsFile/Params6.txt",
		"/home/NetBeansProjects/integration/ParamsFile/Params7.txt",
		"/home/NetBeansProjects/integration/ParamsFile/Params8.txt",
		"/home/NetBeansProjects/integration/ParamsFile/Params9.txt",
		"/home/NetBeansProjects/integration/ParamsFile/Params10.txt",
		"/home/NetBeansProjects/integration/ParamsFile/Params11.txt", };
static float ErreurX[12] = { 0, 0, 0.359079, 0.3557, 0.255017, 0.221681,
		0.154232, 0.191323, 0.237549, 0, 0.352053, 0 };
static float ErreurY[12] = { 0, 0, 0.408716, 0.426033, 0.531639, 0.544478,
		0.541678, 0.472792, 0.489052, 0, 0.43377, 0 };
#endif /* CONSTANTS_H_ */