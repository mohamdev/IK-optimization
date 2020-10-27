/*
 * simple-polynom-ipopt.hpp
 *
 *  Created on: 23 Oct 2020
 *      Author: aladinedev2
 */

#ifndef SIMPLE_POLYNOM_IPOPT_HPP_
#define SIMPLE_POLYNOM_IPOPT_HPP_

#include "IpTNLP.hpp"

#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include <vector>
#include "kinematics.hpp"
using namespace std;
typedef double Scalar;
typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> ScalarMatrix;
typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> ScalarVector;

using namespace Ipopt;


class simplePol_NLP: public Ipopt::TNLP
{
public:
	simplePol_NLP(Scalar const & firstPoint);
	virtual ~simplePol_NLP();
   void setEvalPoint(Scalar const & newEvalPoint);
   vector<Scalar> getEvalTrajectory();
	   virtual bool get_nlp_info(
	      Index&          n,
	      Index&          m,
	      Index&          nnz_jac_g,
	      Index&          nnz_h_lag,
	      IndexStyleEnum& index_style
	   );

	   virtual bool get_bounds_info(
	      Index   n,
	      Number* x_l,
	      Number* x_u,
	      Index   m,
	      Number* g_l,
	      Number* g_u
	   );
	   virtual bool get_starting_point(
	      Index   n,
	      bool    init_x,
	      Number* x,
	      bool    init_z,
	      Number* z_L,
	      Number* z_U,
	      Index   m,
	      bool    init_lambda,
	      Number* lambda
	   );
	   virtual bool eval_f(
	      Index         n,
	      const Number* x,
	      bool          new_x,
	      Number&       obj_value
	   );
	   virtual bool eval_grad_f(
	      Index         n,
	      const Number* x,
	      bool          new_x,
	      Number*       grad_f
	   );
	   virtual bool eval_g(
	      Index         n,
	      const Number* x,
	      bool          new_x,
	      Index         m,
	      Number*       g
	   );
	   virtual bool eval_jac_g(
	      Index         n,
	      const Number* x,
	      bool          new_x,
	      Index         m,
	      Index         nele_jac,
	      Index*        iRow,
	      Index*        jCol,
	      Number*       values
	   );
	   virtual void finalize_solution(
	      SolverReturn               status,
	      Index                      n,
	      const Number*              x,
	      const Number*              z_L,
	      const Number*              z_U,
	      Index                      m,
	      const Number*              g,
	      const Number*              lambda,
	      Number                     obj_value,
	      const IpoptData*           ip_data,
	      IpoptCalculatedQuantities* ip_cq
	   );
private:
   /**@name Methods to block default compiler methods.
    *
    * The compiler automatically generates the following three methods.
    *  Since the default compiler implementation is generally not what
    *  you want (for all but the most simple classes), we usually
    *  put the declarations of these methods in the private section
    *  and never implement them. This prevents the compiler from
    *  implementing an incorrect "default" behavior without us
    *  knowing. (See Scott Meyers book, "Effective C++")
    */
   //@{
   Scalar evalPoint;
   Scalar estPoint;
   vector<Scalar> evalTrajectory;
   simplePol_NLP(
      const simplePol_NLP&
   );

   simplePol_NLP& operator=(
      const simplePol_NLP&
   );
};




#endif /* SIMPLE_POLYNOM_IPOPT_HPP_ */
