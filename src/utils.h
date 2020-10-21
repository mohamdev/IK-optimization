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
#include <urdf_parser/urdf_parser.h>
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/codegen/cppadcg.hpp"
#include <Eigen/Geometry>
#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif


using namespace std;

typedef double Scalar;
typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> ScalarMatrix;
typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> ScalarVector;


void rot2quat(ScalarMatrix const & R, ScalarVector & Q);

void eigen2vector(ScalarMatrix const& eigenMat, std::vector<std::vector<double>> & returnedVect);

#endif /* UTILS_H_ */
