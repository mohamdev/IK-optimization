/*
 * simple-curve-fitting.cpp
 *
 *  Created on: 23 Oct 2020
 *      Author: aladinedev2
 */


//
//#include "utils.h"
//#include <cppad/cppad.hpp> // the CppAD package
//#include <Eigen/Dense>
//#include <Eigen/Core>
//#include <vector>
//#include <stdlib.h>
//#include <random>
//#include <cmath>
//#include <iostream>
//#include <cstdlib>
//#include <string>
//#include <limits>
//#include <fstream>
//#include <time.h>
//#include <ctime>
//#include <math.h>
//#include "pinocchio/parsers/urdf.hpp"
//#include "pinocchio/algorithm/joint-configuration.hpp"
//#include "pinocchio/algorithm/kinematics.hpp"
//#include "pinocchio/autodiff/cppad.hpp"
////#include <urdf_parser/urdf_parser.h>
//#include <pinocchio/algorithm/frames.hpp>
////#include "pinocchio/codegen/cppadcg.hpp"
////#include <cppad/cppad.hpp> // the CppAD package
//#include <Eigen/Geometry>
//
//using namespace CppAD;
//typedef pin::Model Model;
//typedef pin::Model::Data Data;
//typedef double Scalar;
//typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> ScalarMatrix;
//typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> ScalarVector;
//typedef AD<Scalar> ADScalar;
//typedef pin::ModelTpl<ADScalar> ADModel;
//typedef ADModel::Data ADData;
//typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,1> ADVector;
//typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,Eigen::Dynamic> ADMatrix;
//typedef ADModel::ConfigVectorType ADConfigVectorType;

//void setPosADFun(ADModel const & pinADModel, ADData & pinADData){
//	        /**Set an AD configuration ad_q **/
//
//	        ADConfigVectorType ad_q(pinADModel.nv);
//	        ADConfigVectorType & X = ad_q;
//	        CppAD::Independent(X);
//
//	        pin::forwardKinematics(pinADModel, pinADData, ad_q);
//	        pin::updateFramePlacements(pinADModel, pinADData);
//	        ADVector pos(3); pos = pinADData.oMf[5].translation();
//
//	        /**Generate AD function and stop recording **/
//	        CppAD::ADFun<ADScalar> fkine_pos(X,pos);
//
//}

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
#include <cppad/cppad.hpp> // the CppAD package
#include <Eigen/Geometry>

int main(int argc, char ** argv)
{
	using CppAD::AD;
	  using CppAD::NearEqual;

	  typedef double Scalar;
	  typedef AD<Scalar> ADScalar;

	  typedef pinocchio::ModelTpl<Scalar> Model;
	  typedef Model::Data Data;
//	  typedef Model::Motion Motion;

	  typedef pinocchio::ModelTpl<ADScalar> ADModel;
	  typedef ADModel::Data ADData;
	  typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,1> ADVector;
	  typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,1> VectorXAD;
	  Model model;
	  pinocchio::urdf::buildModel("/home/aladinedev2/Desktop/pinocchio-cpp/urdf/human_arm_p.urdf", model);
	  Data data(model);

	  ADModel ad_model = model.cast<ADScalar>();
	  ADData ad_data(ad_model);

	  // Sample random configuration
	  typedef Model::ConfigVectorType ConfigVectorType;
	  typedef ADModel::ConfigVectorType ADConfigVectorType;
//	  typedef Model::TangentVectorType TangentVectorType;
	  ConfigVectorType q(model.nq);
	  q = pinocchio::randomConfiguration(model);

	  ADConfigVectorType ad_q(ad_model.nv);
	ADConfigVectorType & X = ad_q;
	CppAD::Independent(X);

	pinocchio::forwardKinematics(ad_model, ad_data, ad_q);
	pinocchio::updateFramePlacements(ad_model, ad_data);
	ADVector pos(3); pos = ad_data.oMf[5].translation();

	/**Generate AD function and stop recording **/
	CppAD::ADFun<Scalar> fkine_pos(X,pos);



}
