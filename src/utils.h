/*
 * utils.h
 *
 *  Created on: 18 Oct 2020
 *      Author: aladinedev2
 *  This file contains all the typedefs used in the functions of the project.
 *  It also will contain all the needed includes, and all the useful tool functions that
 *  can be used for conversions between different data types.
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>
#include <stdlib.h>
#include <random>
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <string>
#include <limits>
#include <fstream>
#include <time.h>
#include <ctime>
#include <math.h>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
//#include "pinocchio/codegen/cppadcg.hpp"
//#include <urdf_parser/urdf_parser.h>
#include <pinocchio/algorithm/frames.hpp>
//#include "pinocchio/codegen/cppadcg.hpp"
#include "pinocchio/autodiff/cppad.hpp"
#include <cppad/cppad.hpp> // the CppAD package
#include <Eigen/Geometry>
#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif
using namespace CppAD;

using namespace std;
namespace pin = pinocchio;

typedef pin::Model Model;
typedef pin::Model::Data Data;
typedef double Scalar;
typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> ScalarMatrix;
typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> ScalarVector;
typedef AD<Scalar> ADScalar;
typedef pin::ModelTpl<ADScalar> ADModel;
typedef ADModel::Data ADData;
typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,1> ADVector;
typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,Eigen::Dynamic> ADMatrix;
typedef ADModel::ConfigVectorType ADConfigVectorType;

Eigen::Vector4d rot2quat(Eigen::Matrix3d const & R);
void rot2quatAD(ADMatrix const & R, ADVector & Q);
//void getStateVectAD_vel(ADVector & X, ADVector const & q, ADVector const & dq);

void eigen2vector(ScalarMatrix const& eigenMat, std::vector<std::vector<double>> & returnedVect);
void eigen2vector(std::vector<ScalarVector> const& eigenMat, std::vector<std::vector<Scalar>> & returnedVect);

//template <typename eigenVect>
//void x_to_q_dq(ADVector const & Xvel, ADVector & q, ADVector & dq);
template <typename eigenVect>
void x_to_q_dq(eigenVect const & Xvel, eigenVect & q, eigenVect & dq);

template <typename eigenVect>
void x_to_q_dq_ddq(eigenVect const & Xvel, eigenVect & q, eigenVect & dq, eigenVect & ddq);

template <typename eigenVect>
void x_to_q_dq(eigenVect const & Xvel, eigenVect & q, eigenVect & dq)
{
	for(int j = 0; j<q.rows(); j++)
	{
		q(j) = Xvel(j);
		dq(j) = Xvel(j+q.rows());
	}
}

template <typename eigenVect>
void x_to_q_dq_ddq(eigenVect const & Xvel, eigenVect & q, eigenVect & dq, eigenVect & ddq)
{
	for(int j = 0; j<q.rows(); j++)
	{
		q(j) = Xvel(j);
		dq(j) = Xvel(j+q.rows());
		ddq(j) = Xvel(j+2*q.rows());
	}
}
#endif /* UTILS_H_ */
