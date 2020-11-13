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
#include "pinocchio/math/quaternion.hpp"
//#include "pinocchio/codegen/cppadcg.hpp"
//#include <urdf_parser/urdf_parser.h>
#include <pinocchio/algorithm/frames.hpp>
//#include "pinocchio/codegen/cppadcg.hpp"
#include "pinocchio/autodiff/cppad.hpp"
#include <cppad/cppad.hpp> // the CppAD package
#include <Eigen/Geometry>
#include "matplotlibcpp.h"
#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif
using namespace CppAD;

using namespace std;
namespace plt = matplotlibcpp;
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
typedef ADModel::TangentVectorType ADTangentVectorType;

void rotMatToQuaternion(ADMatrix const & R, ADVector & Quat);
Eigen::Vector4d rot2quat(Eigen::Matrix3d const & R);
void rot2quatAD(ADMatrix const & R, ADVector & Q);
//void getStateVectAD_vel(ADVector & X, ADVector const & q, ADVector const & dq);

void eigen2vector(ScalarMatrix const& eigenMat, std::vector<std::vector<double>> & returnedVect);
void eigen2vector(std::vector<ScalarVector> const& eigenMat, std::vector<std::vector<Scalar>> & returnedVect);
void vector2eigen(std::vector<ScalarVector> const& vectorMat, ScalarMatrix & eigenMat);
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

template <typename eigenVect>
void q_dq_to_x(eigenVect & Xvel, eigenVect const & q, eigenVect const & dq);

template <typename eigenVect>
void q_dq_to_x(eigenVect & Xvel, eigenVect const & q, eigenVect const & dq)
{
	for(int j = 0; j<q.rows(); j++)
	{
		Xvel(j) = q(j);
		Xvel(j+q.rows()) = dq(j);
	}
}

template <typename eigenVect>
void q_dq_ddq_to_x(eigenVect & Xvel, eigenVect const & q, eigenVect const & dq, eigenVect const & ddq);

template <typename eigenVect>
void q_dq_ddq_to_x(eigenVect & Xacc, eigenVect const & q, eigenVect const & dq, eigenVect const & ddq)
{
	for(int j = 0; j<q.rows(); j++)
	{
		Xacc(j) = q(j);
		Xacc(j+q.rows()) = dq(j);
		Xacc(j+q.rows()*2) = ddq(j);
	}
}

template<typename Base>
Eigen::Matrix<Base,Eigen::Dynamic,1> rot2quat_tmpl(Eigen::Matrix<Base,Eigen::Dynamic,Eigen::Dynamic> const & m);

template<typename Base>
Eigen::Matrix<Base,Eigen::Dynamic,1> rot2quat_tmpl(Eigen::Matrix<Base,Eigen::Dynamic,Eigen::Dynamic> const & m){
    Base qw = CppAD::sqrt(1.0+m(0,0)+m(1,1)+m(2,2))/2.0;
    Base qx = (m(2,1) - m(1,2))/( 4.0 *qw);
    Base qy = (m(0,2) - m(2,0))/( 4.0 *qw);
    Base qz = (m(1,0) - m(0,1))/( 4.0 *qw);
    Eigen::Matrix<Base,Eigen::Dynamic,1> quat(4,1);
    quat(0) = qw;
    quat(1) = qx;
    quat(2) = qy;
    quat(3) = qz;
    return quat;
};

void plotData(ScalarMatrix const & Q_estimated, ScalarMatrix const & Q_reference, ScalarMatrix const & estimatedMeasurement, ScalarMatrix const & referenceMeasurement, ScalarMatrix const & sensorMeasurement);

ScalarMatrix read_data(std::string const & filePath, std::string const & dataType);
std::fstream& GotoLine(std::fstream& file, unsigned int const & num);
ScalarMatrix readTrajFromCSV(int const & trajIndex, std::string const & typeTraj);
#endif /* UTILS_H_ */
