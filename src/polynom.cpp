/*
 * polynom.cpp
 *
 *  Created on: 18 Oct 2020
 *      Author: aladinedev2
 *
 *  This file contains all the functions and structures that can be used to instanciate, generate, modify and identify a polynom.
 *  The polynoms are based on the "polynom" structure described below.
 *
 */
#include "polynom.h"

int Polynom::getOrder() const{
	return this->n;
};
void Polynom::setOrder(int const & order){
	this->n = order;
};

int Polynom::getDim() const {
	return this->dim;
};
void Polynom::setDim(int const & dimension) {
	this->dim = dimension;
};

ScalarMatrix Polynom::getCoefs() const{
	return this->coefs;
};
void Polynom::setCoefs(ScalarMatrix const & coefficients){
	this->coefs = coefficients;
};

ScalarMatrix Polynom::getTraj() const{
	return this->traj;
};
void Polynom::setTraj(ScalarMatrix const & trajectory){
	this->traj = trajectory;
};

void Polynom::generateRandTraj(double const & dt, double const & duration, int const & nb_points){
	/*
	 * This function generates a random quintic polynomial trajectory between N(=nb_points) points
	 * INPUTS:
	 * 	dt = sampling time
	 * 	duration = time duration between two successive points
	 * 	nb_points = total number of points crossed by the trajectory
	 * OUTPUT:
	 *
	 * */
	ScalarMatrix traj = ScalarMatrix::Zero(1, floor(nb_points*duration/dt));
	double pointsBound = 7.0;

	double t = 0.0;
	double tf = duration;
	int trajIndex = 0;
	std::random_device rd;
	std::mt19937 pointGenerator(rd());
	double endPoint = 0.0;
	double startPoint = 0.0;
	for (int indexPoint = 0; indexPoint < nb_points; indexPoint++)
	{
		t = 0.0;
		std::uniform_real_distribution<double> generatePoint(-pointsBound, pointsBound);
		if (trajIndex == 0)
		{
			startPoint = generatePoint(pointGenerator);
			endPoint = generatePoint(pointGenerator);
		}else
		{
			startPoint = endPoint;
			endPoint = generatePoint(pointGenerator);
		}


		while(t<tf && trajIndex < traj.cols())
		{
			traj(0, trajIndex) = getPolValue(t, startPoint, endPoint, tf);
			t += dt;
			trajIndex++;
		}
		tf += duration;
	}
	this->traj = traj;
	this->n = 5;
};

Scalar getPolValue(double const & t, int const & order,  ScalarVector const & coefs)
{
	/*
	 * This function calculates the value of a polynom of a certain order, at a certain time sample with given coefficients
	 * INPUTS:
	 * 	order = the order of the polynom
	 * 	t = the value of time at which we evaluate the polynom
	 * 	coefs = the vector containing the coefs of the polynom
	 * OUTPUTS:
	 * 	polValue = the value of the polynom at the chosen time value
	 * */
	Scalar polValue = 0;

	for (int n=0; n<order; n++)
	{
		polValue += pow(t,n)*coefs(n);
	}
	return polValue;
}

Scalar getPolValue(double const& t, double const& startPoint, double const& endPoint, double const& finalT)
{
	/*This function evaluates the value of a polynom of deg=5 between 2 points at a given moment of time
	 *INPUTS:
	 *	t = the value of the time at which we evaluate the polynom
	 *	startPoint = the starting point of the polynom's trajectory
	 *	endPoint = the ending point of the polynom's trajectory
	 *	finalT = the value of the time when the trajectory will reach the ending point
	 *OUTPUTS:
	 *	polValue = the value of the polynom at the chosen time value
	 * */

	Scalar polValue = 0;
	Scalar D = endPoint - startPoint;
	polValue = endPoint - (1 - 10*std::pow((t/finalT),3) + 15*pow((t/finalT),4) - 6*pow((t/finalT),5))*D;
	return polValue;
}

polynom generateRandomPol(int const & order, Scalar const & dt, Scalar const & duration){
	/*
	 * This function is used to generate a polynom structure with a chosen order, and random trajectory
	 * for a chosen duration with a chosen sampling time
	 * INPUTS :
	 * 	order = the order of the desired polynom
	 * 	dt = sampling time in seconds
	 * 	duration = the duration of the trajectory in seconds
	 * OUTPUT :
	 * 	generatedPol = polynom structure instance with a trajectory of a chosen duration
	 * */
	double firstCoefsBound = 3.0; //The coefs a0, a1 and a2 will be randomly chosen in the interval [- firstCoefsBound, + firstCoefsBound]
	double otherCoefsBound = 0.001; //The coefs a3 to aN will be randomly chosen in the interval [- otherCoefsBound, + otherCoefsBound]


	polynom generatedPol; //create a generic polynom structure
	generatedPol.n = order; //set its order

	ScalarVector coefs = ScalarMatrix::Zero(order+1,1);  //Initialise to 0 the vector of polynomial coefs

	/*Randomly generate the coefficients of the polynom*/
	std::random_device rd;
	std::mt19937 coefGenerator(rd());
	for(int indexCoef=0; indexCoef<order+1; indexCoef++)
	{
		if (indexCoef < 3)
		{
			std::uniform_real_distribution<double> distrCoef(-firstCoefsBound,firstCoefsBound);
			coefs(indexCoef) = distrCoef(coefGenerator);
		}else
		{
			std::uniform_real_distribution<double> distrCoef(-otherCoefsBound,otherCoefsBound);
			coefs(indexCoef) = distrCoef(coefGenerator);
		}
	}
	generatedPol.coefs = coefs;

	/*Generate the trajectory of the polynom*/
	double t = 0.0; //time variable
	unsigned int i = 0; //Index for trajectory matrix
	generatedPol.traj = ScalarMatrix::Zero(1,floor(duration/dt));
	while (i<generatedPol.traj.cols())
	{
		generatedPol.traj(0,i) = getPolValue(t, order, coefs);
		t += dt;
		i++;
	}
	return generatedPol;
}


polynom generateRandomPol(double const & dt, double const & duration, int const & nb_points)
{
	/*
	 * This function generates a random quintic polynomial trajectory between N(=nb_points) points
	 * INPUTS:
	 * 	dt = sampling time
	 * 	duration = time duration between two successive points
	 * 	nb_points = total number of points crossed by the trajectory
	 * OUTPUT:
	 *
	 * */
	ScalarMatrix traj = ScalarMatrix::Zero(1, floor(nb_points*duration/dt));
	double pointsBound = 7.0;
	polynom pol;

	double t = 0.0;
	double tf = duration;
	int trajIndex = 0;
	std::random_device rd;
	std::mt19937 pointGenerator(rd());
	double endPoint = 0.0;
	double startPoint = 0.0;
	for (int indexPoint = 0; indexPoint < nb_points; indexPoint++)
	{
		t = 0.0;
		std::uniform_real_distribution<double> generatePoint(-pointsBound, pointsBound);
		if (trajIndex == 0)
		{
			startPoint = generatePoint(pointGenerator);
			endPoint = generatePoint(pointGenerator);
		}else
		{
			startPoint = endPoint;
			endPoint = generatePoint(pointGenerator);
		}


		while(t<tf && trajIndex < traj.cols())
		{
			traj(0, trajIndex) = getPolValue(t, startPoint, endPoint, tf);
			t += dt;
			trajIndex++;
		}
		tf += duration;
	}
	pol.traj = traj;
	pol.n = 5;
	return pol;
}




