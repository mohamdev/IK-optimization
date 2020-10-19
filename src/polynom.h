/*
 * polynom.h
 *
 *  Created on: 18 Oct 2020
 *      Author: aladinedev2
 *  This file contains all the functions and structures that can be used to instanciate, generate, modify and identify a polynom.
 *  The polynoms are based on the "polynom" structure described in polynom.cpp.
 */

#ifndef POLYNOM_H_
#define POLYNOM_H_

#include "utils.h"
#include <random>
#include <cmath>

typedef struct _polynom{
	/*
	 * This structure represents a polynom with 3 attributes :
	 * n = the order of the polynom
	 * coefs = the vector which contains the n+1 coefficients of the polynom
	 * traj = contains the trajectory of the polynom during a certain duration
	 * */
	int n;
	ScalarVector coefs;
	ScalarMatrix traj;
}polynom;

Scalar getPolValue(double const & t, int const & order, ScalarVector const & coefs);
Scalar getPolValue(double const& t, double const& startPoint, double const& endPoint, double const& finalT);

polynom generateRandomPol(int const & order, Scalar const & dt, Scalar const & duration);
polynom generateRandomPol(double const & dt, double const & duration, int const & nb_points);


#endif /* POLYNOM_H_ */
