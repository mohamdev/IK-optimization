/*
 * hs071_main.cpp
 *
 *  Created on: 21 Oct 2020
 *      Author: aladinedev2
 */
#include "IpIpoptApplication.hpp"
//#include "simple-polynom-ipopt.hpp"
#include "limb-simple-ipopt.hpp"
#include "polynom.h"
#define dT 0.01
#include <iostream>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

using namespace Ipopt;

int main(
   int    /*argv*/,
   char** /*argc*/
)
{
   // Create a new instance of your nlp
   //  (use a SmartPtr, not raw)
//	   Polynom pol1(5,1);
//	   pol1.generateRandTraj(dT, 0.1, 200);
//	   ScalarMatrix traj;
//	   traj = pol1.getTraj();
   SmartPtr<limb_NLP> mynlp = new limb_NLP("/home/aladinedev2/Desktop/Final_URDF/human_arm_dorent.urdf", 13, 39, 3);


   // Create a new instance of IpoptApplication
   //  (use a SmartPtr, not raw)
   // We are using the factory, since this allows us to compile this
   // example with an Ipopt Windows DLL
   SmartPtr<IpoptApplication> app = new IpoptApplication(); //= new IpoptApplication();//IpoptApplicationFactory();

   // Change some options
   // Note: The following choices are only examples, they might not be
   //       suitable for your optimization problem.
   app->Options()->SetNumericValue("tol", 1e-7);
   app->Options()->SetIntegerValue("print_level", 0);
   app->Options()->SetStringValue("mu_strategy", "adaptive");
//   app->Options()->SetStringValue("output_file", "ipopt.out");
   app->Options()->SetStringValue("hessian_approximation", "limited-memory");
   app->Options()->SetStringValue("derivative_test", "first-order");
//   app->Options()->SetStringValue("derivative_test_print_all", "yes");
   // The following overwrites the default name (ipopt.opt) of the options file
   // app->Options()->SetStringValue("option_file_name", "hs071.opt");

   // Initialize the IpoptApplication and process the options
   ApplicationReturnStatus status;
   status = app->Initialize();
   if( status != Solve_Succeeded )
   {
      std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
      return (int) status;
   }


   std::vector<Scalar> t;
   std::vector<Scalar> refTraj;
   for(int i = 0;  i<mynlp->pol.getTraj().cols()-1; i++)
   {
	   std::cout << (((double)i)/((double)mynlp->pol.getTraj().cols()))*100.0<< " pourcents" << std::endl;
	   status = app->OptimizeTNLP(mynlp);
   }
   Scalar finalResidual = 0;
   for(unsigned int i =0; i<mynlp->residuals.size(); i++){
	   finalResidual += mynlp->residuals[i];
   }
   finalResidual = finalResidual/((double)mynlp->residuals.size());
   std::cout << "Final Residual : " << finalResidual << std::endl;
   finalResidual = std::sqrt(finalResidual);
   std::cout << "RMSE = " << finalResidual << std::endl;
   // Ask Ipopt to solve the problem
   ScalarMatrix q_est_traj = ScalarMatrix::Zero(mynlp->limb.estTraj.qTraj[0].rows(), mynlp->limb.estTraj.qTraj.size());
   ScalarMatrix q_ref_traj = ScalarMatrix::Zero(mynlp->limb.estTraj.qTraj[0].rows(), mynlp->limb.estTraj.qTraj.size());
   ScalarMatrix estMeasTraj = ScalarMatrix::Zero(mynlp->limb.estTraj.measTraj[0].rows(), mynlp->pol.getTraj().cols()-1) ;
   ScalarMatrix refMeasTraj = ScalarMatrix::Zero(mynlp->limb.estTraj.measTraj[0].rows(), mynlp->pol.getTraj().cols()-1);

   vector2eigen(mynlp->limb.estTraj.qTraj, q_est_traj);
   vector2eigen(mynlp->limb.refTraj.qTraj, q_ref_traj);
   vector2eigen(mynlp->limb.estTraj.measTraj, estMeasTraj);
   vector2eigen(mynlp->limb.refTraj.measTraj, refMeasTraj);
   plotData(q_est_traj, q_ref_traj, estMeasTraj, refMeasTraj,  estMeasTraj);
   // As the SmartPtrs go out of scope, the reference count
   // will be decremented and the objects will automatically
   // be deleted.

   return (int) status;
}
// [MAIN]



