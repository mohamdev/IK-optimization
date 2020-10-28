/*
 * hs071_main.cpp
 *
 *  Created on: 21 Oct 2020
 *      Author: aladinedev2
 */
#include "IpIpoptApplication.hpp"
#include "simple-polynom-ipopt.hpp"
#include "polynom.h"
#define dT 0.01
#include <iostream>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

using namespace Ipopt;

//int main(
//   int    /*argv*/,
//   char** /*argc*/
//)
//{
//   // Create a new instance of your nlp
//   //  (use a SmartPtr, not raw)
//	   Polynom pol1(5,1);
//	   pol1.generateRandTraj(dT, 0.1, 200);
//	   ScalarMatrix traj;
//	   traj = pol1.getTraj();
//   SmartPtr<simplePol_NLP> mynlp = new simplePol_NLP(traj(0,0));
//
//
//   // Create a new instance of IpoptApplication
//   //  (use a SmartPtr, not raw)
//   // We are using the factory, since this allows us to compile this
//   // example with an Ipopt Windows DLL
//   SmartPtr<IpoptApplication> app = new IpoptApplication(); //= new IpoptApplication();//IpoptApplicationFactory();
//
//   // Change some options
//   // Note: The following choices are only examples, they might not be
//   //       suitable for your optimization problem.
//   app->Options()->SetNumericValue("tol", 1e-7);
//   app->Options()->SetIntegerValue("print_level", 0);
//   app->Options()->SetStringValue("mu_strategy", "adaptive");
////   app->Options()->SetStringValue("output_file", "ipopt.out");
//   app->Options()->SetStringValue("hessian_approximation", "limited-memory");
////   app->Options()->SetStringValue("derivative_test", "first-order");
////   app->Options()->SetStringValue("derivative_test_print_all", "yes");
//   // The following overwrites the default name (ipopt.opt) of the options file
//   // app->Options()->SetStringValue("option_file_name", "hs071.opt");
//
//   // Initialize the IpoptApplication and process the options
//   ApplicationReturnStatus status;
//   status = app->Initialize();
//   if( status != Solve_Succeeded )
//   {
//      std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
//      return (int) status;
//   }
//
//
//   std::vector<Scalar> t;
//   std::vector<Scalar> refTraj;
//   for(int i = 0;  i<traj.cols(); i++)
//   {
//	   refTraj.push_back(traj(0,i));
//	   t.push_back(i);
//	   mynlp->setEvalPoint(traj(0,i));//= new simplePol_NLP(traj(0,i));
//	   status = app->OptimizeTNLP(mynlp);
//   }
//   // Ask Ipopt to solve the problem
//  std::vector<Scalar> estTraj = mynlp->getEvalTrajectory();
//
//  plt::named_plot("refTraj",t ,refTraj);
//  plt::named_plot("estTraj",t ,estTraj, "r--");
//  plt::legend();
//  plt::show();
//
//   // As the SmartPtrs go out of scope, the reference count
//   // will be decremented and the objects will automatically
//   // be deleted.
//
//   return (int) status;
//}
//// [MAIN]



