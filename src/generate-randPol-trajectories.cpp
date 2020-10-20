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

//	polynom pol1;
//	//pol1 = generateRandomPol(5, dT, 10);
//
//
//	pol1 = generateRandomPol(dT, 10, 50);


	Polynom pol1(5,2);
	//pol1.setDim(2);
	pol1.generateRandTraj(dT, 10, 50);
	ScalarMatrix traj;
	traj = pol1.getTraj();

	std::vector<double> t(traj.cols());
	std::vector<double> y(traj.cols());
	std::vector<double> y2(traj.cols());
	for (int i = 0; i<traj.cols(); i++)
	{
		t[i] = i*dT;
		y[i] = traj(0,i);
		y2[i] = traj(1,i);
	}

	plt::plot(t,y);
	plt::plot(t,y2);
	plt::show();
	return 0;
}
