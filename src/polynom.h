/*
 * polynom.h
 *
 *  Created on: 18 Oct 2020
 *      Author: aladinedev2
 *  This file contains all the functions and structures that can be used to instanciate, generate, modify and identify a polynom.
 *  The polynoms are either based on the "polynom" structure or the Polynom class
 */

#ifndef POLYNOM_H_
#define POLYNOM_H_

#include "utils.h"
#include <random>
#include <cmath>


class Polynom{
private:
	int n; //order of the polynom
	int dim; //dimension of the polynom
	ScalarMatrix coefs; //Matrix containing the coeficients
	ScalarMatrix traj; //Matrix containing the
	ScalarVector polValues; //Vector containing the values of the polynom
public:
	int getOrder() const;
	void setOrder(int const & order);

	int getDim() const;
	void setDim(int const & dimension);

	ScalarMatrix getCoefs() const;
	void setCoefs(ScalarMatrix const & coefficients);

	ScalarMatrix getTraj() const;
	void setTraj(ScalarMatrix const & trajectory);
	void generateRandTraj(double const & dt, double const & duration, int const & nb_points);
};

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
