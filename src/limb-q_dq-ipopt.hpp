/*
 * limb-q_dq-ipopt.hpp
 *
 *  Created on: 27 Nov 2020
 *      Author: aladinedev2
 */

#ifndef LIMB_Q_DQ_IPOPT_HPP_
#define LIMB_Q_DQ_IPOPT_HPP_

#include "IpTNLP.hpp"

#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include <vector>
#include "kinematics.hpp"
#include "polynom.h"
#define Te (double)(1.0/60.0)

using namespace std;
typedef double Scalar;
typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> ScalarMatrix;
typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> ScalarVector;

using namespace Ipopt;

class limb_q_dq_NLP: public Ipopt::TNLP
{
public:
	limb_q_dq_NLP(string const & urdf_filename, int const & nb_states, int const & nb_measurements, int const & nb_sensors);
	virtual ~limb_q_dq_NLP();
   void setInitPoint(ScalarVector const & q_est, ScalarVector const & dq_est);
   std::vector<ScalarVector> getEvalTrajectory();
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
	   virtual bool eval_h(
	      Index         n,
	      const Number* x,
	      bool          new_x,
	      Number        obj_factor,
	      Index         m,
	      const Number* lambda,
	      bool          new_lambda,
	      Index         nele_hess,
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
	   ScalarVector initPoint;
	   ScalarVector estPoint;
	   ScalarMatrix refTraj;
	   ScalarMatrix ref_dq_Traj;
	   ScalarMatrix ref_ddq_Traj;
	   Polynom pol;
	   size_t counter;
	   ScalarVector res_q_jac;
	   ScalarVector X_es;
	   std::vector<ScalarVector> evalTrajectory;
	   std::vector<Scalar> residuals;
	   Limb limb;
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
   limb_q_dq_NLP(
      const limb_q_dq_NLP&
   );

   limb_q_dq_NLP& operator=(
      const limb_q_dq_NLP&
   );
};



#endif /* LIMB_Q_DQ_IPOPT_HPP_ */
