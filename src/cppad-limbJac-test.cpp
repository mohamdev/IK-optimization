/*
 * cppad-limbJac-test.cpp
 *
 *  Created on: 4 Nov 2020
 *      Author: aladinedev2
 */


//
//#include "kinematics.hpp"
//#include "polynom.h"
//#include "matplotlibcpp.h"
//#define dT 0.01
//namespace plt = matplotlibcpp;
//
//int main(int argc, char ** argv)
//{
//  using namespace pinocchio;
//
//	//Polynom pol1(5,13);
//	//pol1.setDim(2);
//	//pol1.generateRandTraj(dT, 0.1, 50);
//	ScalarMatrix traj;
//	traj = readTrajFromCSV(2, "pos");//pol1.getTraj();
//  const std::string urdf_filename = "/home/aladinedev2/Desktop/Final_URDF/human_arm_dorent.urdf";
//  int nb_vimu = 3;
//  int nb_joints = 13;
//
//  // Load the urdf model
//  std::cout << "Constructing limb " << std::endl;
//  Limb arm = Limb(urdf_filename, nb_joints, nb_vimu*13, 3); //Instantiate an arm from urdf_filename, nb_states = 13 and nb_measurements = 3
//  std::cout << "Adding sensor " << std::endl;
//  arm.addSensor(13, "IMU1_link");
//  std::cout << "Adding sensor " << std::endl;
//  arm.addSensor(13, "IMU2_link");
//  std::cout << "Adding sensor " << std::endl;
//  arm.addSensor(13, "IMU3_link");
//  std::cout << "Adding sensor " << std::endl;
//
//  ScalarVector q = ScalarMatrix::Zero(13,1);
//  ScalarVector q_es = ScalarMatrix::Zero(13,1);
//  //ScalarMatrix traj_q_es = ScalarMatrix::Zero(traj.cols(), traj.rows());
//  ScalarVector resJac;
//
//  //q(5) = traj(0,0);
//  for (int i =0; i<traj.cols(); i++){
//	  std::cout << "i : " << i << std::endl;
//	  q = traj.col(i);
//	  q_es = q;
////	  for (int j = 0; j<6; j++) q_es(j) = q(j);
////	  for (int j =6; j<13; j++) q_es(j) = q(j) + 0.06;
//
//	  arm.setJointPos(q, REF);
//	  arm.setJointPos(q_es, EST);
//
//	  arm.refreshSensors(REF);
//	  arm.refreshSensors(EST);
//
//	  //arm.setLimb_res_Jacobian();
//
//	  arm.timesample.push_back(i);	  arm.t.push_back(i*dT);
//	  //arm.refreshAllSensors(EST);
//  }
//  resJac = arm.getLimb_res_Jacobian();
//  std::cout << "Jacobian : " << resJac << std::endl;
//  ScalarMatrix q_est_traj = ScalarMatrix::Zero(arm.estTraj.qTraj[0].rows(), arm.estTraj.qTraj.size());
//  ScalarMatrix q_ref_traj = ScalarMatrix::Zero(arm.estTraj.qTraj[0].rows(), arm.estTraj.qTraj.size());
//  ScalarMatrix estMeasTraj = ScalarMatrix::Zero(arm.estTraj.measTraj[0].rows(), traj.cols()) ;
//  ScalarMatrix refMeasTraj = ScalarMatrix::Zero(arm.estTraj.measTraj[0].rows(), traj.cols());
//
//  vector2eigen(arm.estTraj.qTraj, q_est_traj);
//  vector2eigen(arm.refTraj.qTraj, q_ref_traj);
//  vector2eigen(arm.estTraj.measTraj, estMeasTraj);
//  vector2eigen(arm.refTraj.measTraj, refMeasTraj);
//  plotData(q_est_traj, q_ref_traj, estMeasTraj, refMeasTraj,  estMeasTraj);
//  cout << "finish" << endl;
//
// return 0;
//}
