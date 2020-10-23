/*
 * simple-polynom-ipopt.cpp
 *
 *  Created on: 23 Oct 2020
 *      Author: aladinedev2
 */




#include "simple-polynom-ipopt.hpp"
#include "IpTNLP.hpp"
#include <stdlib.h>
#include <iostream>

using namespace Ipopt;

simplePol_NLP::simplePol_NLP(){

}
simplePol_NLP::~simplePol_NLP(){

}
// returns the size of the problem
bool simplePol_NLP::get_nlp_info(
   Index&          n,
   Index&          m,
   Index&          nnz_jac_g,
   Index&          nnz_h_lag,
   IndexStyleEnum& index_style
)
{
   // The problem described in HS071_NLP.hpp has 4 variables, x[0] through x[3]
   n = 1;
   // one equality constraint and one inequality constraint
   m = 2;
   // in this example the jacobian is dense and contains 8 nonzeros
   nnz_jac_g = 2;
   // the Hessian is also dense and has 16 total nonzeros, but we
   // only need the lower left corner (since it is symmetric)
   nnz_h_lag = 0;
   // use the C style indexing (0-based)
   index_style = TNLP::C_STYLE;
   return true;
}

bool simplePol_NLP::get_bounds_info(
   Index   n,
   Number* x_l,
   Number* x_u,
   Index   m,
   Number* g_l,
   Number* g_u
)
{
	   // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
	   // If desired, we could assert to make sure they are what we think they are.
	   n = 1;
	   m = 2;
	   // the variables have lower bounds of 1
	   x_l[0] = -2;
	   x_u[0] = 2;

	   // the first constraint g1 has a lower bound of 25
	   g_l[0] = -1;
	   // the first constraint g1 has NO upper bound, here we set it to 2e19.
	   // Ipopt interprets any number greater than nlp_upper_bound_inf as
	   // infinity. The default value of nlp_upper_bound_inf and nlp_lower_bound_inf
	   // is 1e19 and can be changed through ipopt options.
	   g_u[0] = 1;
	   // the second constraint g2 is an equality constraint, so we set the
	   // upper and lower bound to the same value
	   g_l[1] = -1;
	   g_u[1] = 1;
	   return true;
}

// returns the initial point for the problem
bool simplePol_NLP::get_starting_point(
   Index   n,
   bool    init_x,
   Number* x,
   bool    init_z,
   Number* z_L,
   Number* z_U,
   Index   m,
   bool    init_lambda,
   Number* lambda
)
{
   // Here, we assume we only have starting values for x, if you code
   // your own NLP, you can provide starting values for the dual variables
   // if you wish
   n = 1;
   init_x = true;
   init_z = false;
   init_lambda = false;
   // initialize to the given starting point
   x[0] = 0.5;
   return true;
}

bool simplePol_NLP::eval_f(
   Index         n,
   const Number* x,
   bool          new_x,
   Number&       obj_value
)
{
	n = 1;
	obj_value = std::pow(0.29*std::sin(x[0]) - 0.29*std::sin(0.73),2) + std::pow(-0.29*std::cos(x[0]) + 0.29*std::cos(0.73),2);
			//std::pow(0.29*std::sin(x[0]) - 0.29*std::sin(1.5),2) + std::pow(-0.29*std::cos(x[0]) - (-0.29*std::cos(1.5)),2);
	return true;
}

// return the gradient of the objective function grad_{x} f(x)
bool simplePol_NLP::eval_grad_f(
   Index         n,
   const Number* x,
   bool          new_x,
   Number*       grad_f
)
{
   n = 1 ;
   grad_f[0] = 2*0.29*0.29*cos(x[0])*sin(x[0]) - 2*0.29*0.29*sin(0.73)*cos(x[0]) - 2*0.29*0.29*sin(x[0])*cos(x[0]) + 2*0.29*0.29*cos(0.73)*sin(x[0]);
   return true;
}

// return the value of the constraints: g(x)
bool simplePol_NLP::eval_g(
   Index         n,
   const Number* x,
   bool          new_x,
   Index         m,
   Number*       g
)
{
   n = 1;
   m = 2;
   g[0] = sin(x[0]);
   g[1] = cos(x[0]);
   return true;
}
bool simplePol_NLP::eval_jac_g(
   Index         n,
   const Number* x,
   bool          new_x,
   Index         m,
   Index         nele_jac,
   Index*        iRow,
   Index*        jCol,
   Number*       values
)
{
	n = 1;
	m = 2;
	nele_jac = 2;
	if (values == NULL){
	    iRow[0] = 0;
	    jCol[0] = 0;
	    iRow[1] = 1;
	    jCol[1] = 0;
	}else{
		values[0] = cos(x[0]);
		values[1] = -sin(x[0]);
	}

	return true;
}
void simplePol_NLP::finalize_solution(
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
)
{
	n = 1;
	m = 2;
	   // For this example, we write the solution to the console
	   std::cout << std::endl << std::endl << "Solution of the primal variables, x" << std::endl;
	   for( Index i = 0; i < n; i++ )
	   {
	      std::cout << "x[" << i << "] = " << x[i] << std::endl;
	   }
	   std::cout << std::endl << std::endl << "Solution of the bound multipliers, z_L and z_U" << std::endl;
	   for( Index i = 0; i < n; i++ )
	   {
	      std::cout << "z_L[" << i << "] = " << z_L[i] << std::endl;
	   }
	   for( Index i = 0; i < n; i++ )
	   {
	      std::cout << "z_U[" << i << "] = " << z_U[i] << std::endl;
	   }
	   std::cout << std::endl << std::endl << "Objective value" << std::endl;
	   std::cout << "f(x*) = " << obj_value << std::endl;
	   std::cout << std::endl << "Final value of the constraints:" << std::endl;
	   for( Index i = 0; i < m; i++ )
	   {
	      std::cout << "g(" << i << ") = " << g[i] << std::endl;
	   }
}

