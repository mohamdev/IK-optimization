/*
 * polynom.h
 *
 *  Created on: 18 Oct 2020
 *      Author: aladinedev2
 */

#ifndef POLYNOM_H_
#define POLYNOM_H_

#include "utils.h"
#include <random>
#include <cmath>

typedef struct _polynom polynom;

Scalar getPolValueFromCoefs(int const & order, double const & t, ScalarVector const & coefs);

polynom generateRandomPol(int const & order, Scalar const & dt, Scalar const & duration);

Scalar getPolValueFromPoints(double const & t, double const& startPoint, double const& endPoint, double const& finalT);

ScalarMatrix generateRandPolTraj(double const & dt, double const & duration, int const & nb_points);


#endif /* POLYNOM_H_ */
