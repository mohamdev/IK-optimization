/*
 * limb-simple-ipopt.cpp
 *
 *  Created on: 7 Nov 2020
 *      Author: aladinedev2
 */




#include "limb-simple-ipopt.hpp"
//#include "IpTNLP.hpp"
//#include "kinematics.hpp"
//#include <stdlib.h>
//#include <iostream>

using namespace Ipopt;

limb_NLP::limb_NLP(string const & urdf_filename, int const & nb_states, int const & nb_measurements, int const & nb_sensors){

	//Generate Polynomial Reference Trajectory
	this->pol = Polynom(5,13);
	this->pol.generateRandTraj(Te, 1, 50);
	this->refTraj = this->pol.getTraj();
	//Generate Limb and ADD THE SENSORS
	this->limb = Limb(urdf_filename, nb_states, nb_measurements, nb_sensors);
    this->limb.addSensor(13, "IMU1_link");
    this->limb.addSensor(13, "IMU2_link");
    this->limb.addSensor(13, "IMU3_link");
    this->limb.setResidualCostFunc();
	this->X_es = ScalarMatrix::Zero(39,1);
	this->counter = 0;
	ScalarVector q_init = this->pol.getTraj().col(0).segment(6,7);
	this->setInitPoint(q_init);
}
limb_NLP::~limb_NLP(){

}

void limb_NLP::setInitPoint(ScalarVector const & q_est){
	this->initPoint = q_est;
}

std::vector<ScalarVector> limb_NLP::getEvalTrajectory(){
	return this->evalTrajectory;
}
// returns the size of the problem
bool limb_NLP::get_nlp_info(
   Index&          n,
   Index&          m,
   Index&          nnz_jac_g,
   Index&          nnz_h_lag,
   IndexStyleEnum& index_style
)
{
   // The problem described in HS071_NLP.hpp has 4 variables, x[0] through x[3]
   n = 7;
   // one equality constraint and one inequality constraint
   m = 0;
   // in this example the jacobian is dense and contains 8 nonzeros
   nnz_jac_g = 0;
   // the Hessian is also dense and has 16 total nonzeros, but we
   // only need the lower left corner (since it is symmetric)
   nnz_h_lag = 0;
   // use the C style indexing (0-based)
   index_style = TNLP::C_STYLE;
   return true;
}

bool limb_NLP::get_bounds_info(
   Index   n,
   Number* x_l,
   Number* x_u,
   Index   m,
   Number* g_l,
   Number* g_u
)
{
		//std::cout << "I'm getting bounds info" << std::endl;
	   // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
	   // If desired, we could assert to make sure they are what we think they are.
	   assert(n == 7);
	   assert(m == 0);
//	   // the variables have lower bounds of 1
//	   for (int i = 0; i<n; i++){
//		   x_l[i] = -2.20;
//		   x_u[i] = 2.20;
//	   }
	   // the variables have lower bounds of 1
	   for( Index i = 0; i < n; i++ )
	   {
	      x_l[i] = -2.20;
	   }

	   // the variables have upper bounds of 5
	   for( Index i = 0; i < n; i++ )
	   {
	      x_u[i] = 2.20;
	   }

//	   // the first constraint g1 has a lower bound of 25
//	   g_l[0] = -1;
//	   // the first constraint g1 has NO upper bound, here we set it to 2e19.
//	   // Ipopt interprets any number greater than nlp_upper_bound_inf as
//	   // infinity. The default value of nlp_upper_bound_inf and nlp_lower_bound_inf
//	   // is 1e19 and can be changed through ipopt options.
//	   g_u[0] = 1;
//	   // the second constraint g2 is an equality constraint, so we set the
//	   // upper and lower bound to the same value
//	   g_l[1] = -1;
//	   g_u[1] = 1;
	   return true;
}

// returns the initial point for the problem
bool limb_NLP::get_starting_point(
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
   assert(n == 7);

//   std::cout << " ------------ SETTING START POINT : ------------ " << this->counter << std::endl;
   assert(init_x == true);
   assert(init_z == false);
   assert(init_lambda == false);
   //std::cout << "Getting startPoint" << std::endl;
   // initialize to the given starting point
   for (int i = 0; i<n; i++)
   {
	   x[i] = this->initPoint(i);
	   //x[i] = this->pol.getTraj().col(counter)(i+6);
   }

   return true;
}

bool limb_NLP::eval_f(
   Index         n,
   const Number* x,
   bool          new_x,
   Number&       obj_value
)
{
	assert(n == 7);
//	std::cout << " ------------ EVALUATIING F : ------------ " << this->counter << std::endl;
	this->limb.setJointPos(this->pol.getTraj().col(counter), REFeval);
	   	ScalarVector q_es = ScalarMatrix::Zero(13,1);
	   	q_es = this->limb.refState.q;

	   	for(int i = 0; i<7; i++)
	   	{
	   		q_es(i+6) = x[i];
	   	}
//	   	std::cout << "q_ref      : " << this->limb.refState.q.transpose() << std::endl;
//	   	std::cout << "q_es       : " << q_es.transpose() << std::endl;

	   	this->limb.setJointPos(q_es, ESTeval);
//	   	std::cout << "estState.q : " << this->limb.estState.q.transpose() << std::endl;
	   	this->limb.setJointNumDerivatives(REFeval);
	   	this->limb.setJointNumDerivatives(ESTeval);
	   	this->limb.refreshSensors(REFeval);
	   	this->limb.refreshSensors(ESTeval);
//	   	std::cout << "x : ";
//	    for (int i = 0 ; i < 7; i++)
//	        cout << x[i] << "  ";
//	    std::cout << std::endl;
//	    std::cout << "estState.dq : " << this->limb.estState.dq.transpose() << std::endl;
//	    std::cout << "refState.dq : " << this->limb.estState.dq.transpose() << std::endl;
//	    std::cout << "estState.ddq : " << this->limb.estState.ddq.transpose() << std::endl;
//	    std::cout << "refState.ddq : " << this->limb.estState.ddq.transpose() << std::endl;
//	    std::cout << "refMeas : " << this->limb.getMeas(REF).transpose() << std::endl;
//	    std::cout << "estMeas : " << this->limb.getMeas(EST).transpose() << std::endl;
	//q_dq_ddq_to_x<ScalarVector>(this->X_es, this->limb.estState.q, this->limb.estState.dq, this->limb.estState.ddq);
	//obj_value = (this->limb.getResidual(this->X_es, this->limb.getSensorsPos(EST)))(0);
	   obj_value = (this->limb.getResidual(q_es, this->limb.getSensorsPos(REF)))(0);
//	std::cout << "residual : " << obj_value << std::endl;
			//std::pow(0.29*std::sin(x[0]) - 0.29*std::sin(1.5),2) + std::pow(-0.29*std::cos(x[0]) - (-0.29*std::cos(1.5)),2);
	return true;
}

// return the gradient of the objective function grad_{x} f(x)
bool limb_NLP::eval_grad_f(
   Index         n,
   const Number* x,
   bool          new_x,
   Number*       grad_f
)
{
   assert(n == 7);
//   std::cout << "------------ EVALUATIING GRADIENT OF F : ------------" << this->counter << std::endl;
   this->limb.setJointPos(this->pol.getTraj().col(counter), REFeval);
  	ScalarVector q_es = ScalarMatrix::Zero(13,1);
  	q_es = this->limb.refState.q;
   	for(int i = 0; i<7; i++)
   	{
   		q_es(i+6) = x[i];
   	}
//   	std::cout << "q_ref      : " << this->limb.refState.q.transpose() << std::endl;
//   	std::cout << "q_es       : " << q_es.transpose() << std::endl;

   	this->limb.setJointPos(q_es, ESTeval);
//   	std::cout << "estState.q : " << this->limb.estState.q.transpose() << std::endl;
   	this->limb.setJointNumDerivatives(REFeval);
   	this->limb.setJointNumDerivatives(ESTeval);
   	this->limb.refreshSensors(REFeval);
   	this->limb.refreshSensors(ESTeval);
//   	std::cout << "x : ";
//    for (int i = 0 ; i < 7; i++)
//        cout << x[i] << "  ";
//    std::cout << std::endl;
//    std::cout << "estState.dq : " << this->limb.estState.dq.transpose() << std::endl;
//    std::cout << "refState.dq : " << this->limb.estState.dq.transpose() << std::endl;
//    std::cout << "estState.ddq : " << this->limb.estState.ddq.transpose() << std::endl;
//    std::cout << "refState.ddq : " << this->limb.estState.ddq.transpose() << std::endl;
//    std::cout << "refMeas : " << this->limb.getMeas(REF).transpose() << std::endl;
//    std::cout << "estMeas : " << this->limb.getMeas(EST).transpose() << std::endl;
   	//q_dq_ddq_to_x<ScalarVector>(this->X_es, this->limb.estState.q, this->limb.estState.dq, this->limb.estState.ddq);
//   	std::cout << "res_q_jac BEFORE : " << this->res_q_jac.transpose() << std::endl;
   //this->res_q_jac = this->limb.getResidualJacobian(this->X_es, this->limb.getMeas(REF));
   	ScalarVector newJac = ScalarMatrix::Zero(13,1);
   //this->res_q_jac = this->limb.getResidualJacobian(this->X_es, this->limb.getMeas(REF));
   	newJac = this->limb.getResidualJacobian(q_es, this->limb.getSensorsPos(REF));
//   std::cout << "res_q_jac AFTER  : " << this->res_q_jac.transpose() << std::endl;
   for (int i =0; i<7; i++){
	   grad_f[i] = newJac(i+6);
   }
//   grad_f[0] = 1;
//   grad_f[1] = 2;
//   grad_f[3] = 0.2;
//   grad_f[4] = 0.21;
//   grad_f[5] = 0.48;
//   grad_f[6] = 1.22;
//   std::cout << "grad_f : " << grad_f[0] << " "
//	<< grad_f[1] << " "
//	<< grad_f[2] << " "
//	<< grad_f[3] << " "
//	<< grad_f[4] << " "
//	<< grad_f[5] << " "
//	<< grad_f[6] << " " << std::endl;
   //grad_f[0] = 2*0.29*0.29*cos(x[0])*sin(x[0]) - 2*0.29*0.29*sin(evalPoint)*cos(x[0]) - 2*0.29*0.29*sin(x[0])*cos(x[0]) + 2*0.29*0.29*cos(evalPoint)*sin(x[0]);
   return true;
}

// return the value of the constraints: g(x)
bool limb_NLP::eval_g(
   Index         n,
   const Number* x,
   bool          new_x,
   Index         m,
   Number*       g
)
{
   assert(n == 7);
   assert(m == 0);
   //std::cout << "I'm evaluating g" << std::endl;
//   g[0] = sin(x[0]);
//   g[1] = cos(x[0]);
   return true;
}
bool limb_NLP::eval_jac_g(
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
	assert(n == 7);
	assert(m == 0);
	nele_jac = 0;
	//std::cout << "I'm evaluating gradient of g" << std::endl;
//	if (values == NULL){
//	    iRow[0] = 0;
//	    jCol[0] = 0;
//	    iRow[1] = 1;
//	    jCol[1] = 0;
//	}else{
//		values[0] = cos(x[0]);
//		values[1] = -sin(x[0]);
//	}

	return true;
}
bool limb_NLP::eval_h(
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
)
{
   assert(n == 7);
   assert(m == 0);


   return true;
}
// [TNLP_eval_h]

void limb_NLP::finalize_solution(
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
//	n = 7;
//	m = 0;
	//std::cout << "I'm finalizing solution" << std::endl;
	this->limb.setJointPos(this->pol.getTraj().col(counter), REF);
	ScalarVector q_es = this->limb.refState.q;
	for(int i = 0; i<7; i++)
	{
		q_es(i+6) = x[i];
	}

	this->limb.setJointPos(q_es, EST);
	this->limb.setJointNumDerivatives();

	this->limb.refreshSensors(REF);
	this->limb.refreshSensors(EST);

	q_dq_ddq_to_x<ScalarVector>(this->X_es, this->limb.estState.q, this->limb.estState.dq, this->limb.estState.ddq);
   for (int i = 0; i<n; i++)
   {
	    this->initPoint(i) = x[i];
   }
   this->limb.timesample.push_back(counter);
   this->limb.t.push_back(counter*Te);
   this->counter++;
   std::cout << "COUNTER VALUE : " << this->counter << std::endl;

//
////	this->estPoint = x[0];
////	this->evalTrajectory.push_back(x[0]);
//
//
//
//   	   std::cout << "Ref dq : " << this->limb.refState.q.transpose() << std::endl;
//	   // For this example, we write the solution to the console
//	   std::cout << std::endl << std::endl << "Solution of the primal variables, x" << std::endl;
//	   for( Index i = 0; i < n; i++ )
//	   {
//	      std::cout << "x[" << i << "] = " << x[i] << std::endl;
//	   }
//	   std::cout << std::endl << std::endl << "Solution of the bound multipliers, z_L and z_U" << std::endl;
//	   std::cout << std::endl << std::endl << "Objective value" << std::endl;
//	   std::cout << "f(x*) = " << obj_value << std::endl;
	   this->residuals.push_back(obj_value);
}

