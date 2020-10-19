//============================================================================
// Name        : ik-optimization.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "matplotlibcpp.h"
#include "polynom.h"
#define dT 0.01

using namespace std;
namespace plt = matplotlibcpp;

int main() {

	ScalarMatrix traj;
	traj = generateRandPolTraj(dT, 10, 50);

	std::vector<double> t(traj.cols());
	std::vector<double> y(traj.cols());

	for (int i = 0; i<traj.cols(); i++)
	{
		t[i] = i*dT;
		y[i] = traj(0,i);
	}

	plt::plot(t,y);
	plt::show();
	return 0;
}
