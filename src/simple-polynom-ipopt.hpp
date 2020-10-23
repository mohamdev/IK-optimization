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

using namespace Ipopt;

class simplePol_NLP: public TNLP
{
public:
	   virtual bool get_nlp_info(
	      Index&          n,
	      Index&          m,
	      Index&          nnz_jac_g,
	      Index&          nnz_h_lag,
	      IndexStyleEnum& index_style
	   ) = 0;

	   virtual bool get_bounds_info(
	      Index   n,
	      Number* x_l,
	      Number* x_u,
	      Index   m,
	      Number* g_l,
	      Number* g_u
	   ) = 0;
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
	   ) = 0;
	   virtual bool eval_f(
	      Index         n,
	      const Number* x,
	      bool          new_x,
	      Number&       obj_value
	   ) = 0;
	   virtual bool eval_grad_f(
	      Index         n,
	      const Number* x,
	      bool          new_x,
	      Number*       grad_f
	   ) = 0;
	   virtual bool eval_g(
	      Index         n,
	      const Number* x,
	      bool          new_x,
	      Index         m,
	      Number*       g
	   ) = 0;
};



#endif /* SIMPLE_POLYNOM_IPOPT_HPP_ */
