/*
 * cppad-limbJac-test.cpp
 *
 *  Created on: 4 Nov 2020
 *      Author: aladinedev2
 */



//#include "kinematics.hpp"
//#include "polynom.h"
//#include "matplotlibcpp.h"
//#define dT (double)(1.0/60.0)
//namespace plt = matplotlibcpp;
//
//int main(int argc, char ** argv)
//{
//  using namespace pinocchio;
//
//	//Polynom pol1(5,13);
//	//pol1.setDim(2);
//	//pol1.generateRandTraj(dT, 0.1, 50);
//  	//Scalar Te = 1.0 / 60.0;
//	ScalarMatrix traj;
//	traj = readTrajFromCSV(2, "pos");//pol1.getTraj();
//	ScalarMatrix dqTraj = readTrajFromCSV(2, "vel");
//	ScalarMatrix ddqTraj = readTrajFromCSV(2, "acc");
//
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
//  ScalarMatrix traj_q_es = ScalarMatrix::Zero(traj.cols(), traj.rows());
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
////	  if(i < 2){
////		  arm.setJointVel(dqTraj.col(i), REF);
////		  arm.setJointVel(dqTraj.col(i), EST);
////		  arm.setJointAcc(ddqTraj.col(i), REF);
////		  arm.setJointAcc(ddqTraj.col(i), EST);
////	  }
//      arm.setJointNumDerivatives(REF);
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
//  ScalarMatrix dq_est_traj = ScalarMatrix::Zero(arm.estTraj.qTraj[0].rows(), arm.estTraj.qTraj.size());
//  ScalarMatrix ddq_est_traj = ScalarMatrix::Zero(arm.estTraj.qTraj[0].rows(), arm.estTraj.qTraj.size());
//  ScalarMatrix estMeasTraj = ScalarMatrix::Zero(arm.estTraj.measTraj[0].rows(), traj.cols()) ;
//  ScalarMatrix refMeasTraj = ScalarMatrix::Zero(arm.estTraj.measTraj[0].rows(), traj.cols());
//
////  vector2eigen(arm.estTraj.qTraj, q_est_traj);
////  vector2eigen(arm.refTraj.qTraj, q_ref_traj);
//  vector2eigen(arm.refTraj.dqTraj, dq_est_traj);
//  vector2eigen(arm.refTraj.ddqTraj, ddq_est_traj);
////  vector2eigen(arm.estTraj.measTraj, estMeasTraj);
////  vector2eigen(arm.refTraj.measTraj, refMeasTraj);
////  plotData(q_est_traj, q_ref_traj, estMeasTraj, refMeasTraj,  estMeasTraj);
//
//  std::vector<std::vector<double>> q_ref(arm.estTraj.qTraj[0].rows(), std::vector<double>(arm.estTraj.qTraj.size())); //Reference angles
//  std::vector<std::vector<double>> dq_ref(arm.estTraj.qTraj[0].rows(), std::vector<double>(arm.estTraj.qTraj.size())); //Estimated angles
//  std::vector<std::vector<double>> ddq_ref(arm.estTraj.qTraj[0].rows(), std::vector<double>(arm.estTraj.qTraj.size())); //Estimated angles
//
//  std::vector<std::vector<double>> dq_es(arm.estTraj.qTraj[0].rows(), std::vector<double>(arm.estTraj.qTraj.size())); //Estimated angles
//  std::vector<std::vector<double>> ddq_es(arm.estTraj.qTraj[0].rows(), std::vector<double>(arm.estTraj.qTraj.size())); //Estimated angles
//  eigen2vector(q_est_traj, q_ref); //COnvert estimatedData to vector
//  eigen2vector(dqTraj, dq_ref); //Convert q_data to vector
//  eigen2vector(ddqTraj, ddq_ref); //Convert q_data to vector
//
//  eigen2vector(dq_est_traj, dq_es);
//  eigen2vector(ddq_est_traj, ddq_es);
//
//  plt::figure(1);
//  for(unsigned int i = 1; i<14; i++)
//  {
//	  plt::subplot(13,1,i);
//	  plt::plot(arm.timesample, dq_ref[i-1], "g");
//	  plt::plot(arm.timesample, dq_es[i-1], "r--");
//  }
//  plt::figure(2);
//  for(unsigned int i = 1; i<14; i++)
//  {
//	  plt::subplot(13,1,i);
//	  plt::plot(arm.timesample, ddq_ref[i-1], "g");
//	  plt::plot(arm.timesample, ddq_es[i-1], "r--");
//  }
//  plt::show();
//  cout << "finish" << endl;
//
// return 0;
//}

//  std::vector<int> timesample;
//  std::vector<Scalar> t;
//  for (int i =0; i<traj.cols(); i++){
//	  if(i<1){
//		  dq_est_traj.col(i) = dqTraj.col(i);
//		  ddq_est_traj.col(i) = ddqTraj.col(i);
//	  }else{
//		  dq_est_traj.col(i) = (traj.col(i) - traj.col(i-1))/dT;
//		  ddq_est_traj.col(i) = (dq_est_traj.col(i) - dq_est_traj.col(i-1))/dT;
//	  }
//	  timesample.push_back(i);
//	  t.push_back(i*dT);
//  }
//  std::vector<std::vector<double>> dq_ref(dqTraj.rows(), std::vector<double>(dqTraj.cols())); //Estimated angles
//  std::vector<std::vector<double>> ddq_ref(ddqTraj.rows(), std::vector<double>(ddqTraj.cols())); //Estimated angles
//
//  std::vector<std::vector<double>> dq_es(dqTraj.rows(), std::vector<double>(dqTraj.cols())); //Estimated angles
//  std::vector<std::vector<double>> ddq_es(ddqTraj.rows(), std::vector<double>(ddqTraj.cols())); //Estimated angles
//
//  eigen2vector(dqTraj, dq_ref); //Convert q_data to vector
//  eigen2vector(ddqTraj, ddq_ref); //Convert q_data to vector
//
//  eigen2vector(dq_est_traj, dq_es);
//  eigen2vector(ddq_est_traj, ddq_es);
//
//  plt::figure(1);
//  for(unsigned int i = 1; i<14; i++)
//  {
//	  plt::subplot(13,1,i);
//	  plt::plot(timesample, dq_ref[i-1], "g");
//	  plt::plot(timesample, dq_es[i-1], "r--");
//  }
//
//  plt::figure(2);
//  for(unsigned int i = 1; i<14; i++)
//  {
//	  plt::subplot(13,1,i);
//	  plt::plot(timesample, ddq_ref[i-1], "g");
//	  plt::plot(timesample, ddq_es[i-1], "r--");
//  }
//  plt::show();
//  cout << "finish" << endl;
