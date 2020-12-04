/*
 * simple-forward-kinematics.cpp
 *
 *  Created on: 23 Oct 2020
 *      Author: aladinedev2
 */

//#include "kinematics.hpp"
//
//
//#include "pinocchio/parsers/urdf.hpp"
//#include "pinocchio/algorithm/joint-configuration.hpp"
//#include "pinocchio/algorithm/kinematics.hpp"
//#include "pinocchio/algorithm/kinematics-derivatives.hpp"
//#include "pinocchio/algorithm/frames.hpp"
//#include "pinocchio/algorithm/frames-derivatives.hpp"
//
//
//int main(int argc, char ** argv)
//{
//  using namespace pinocchio;
//  const std::string urdf_filename = "/home/aladinedev2/Desktop/Final_URDF/human_arm_dorent.urdf";
//
//	ScalarMatrix qTraj = readTrajFromCSV(2, "pos");
//	ScalarMatrix dqTraj = readTrajFromCSV(2, "vel");
//	ScalarMatrix ddqTraj = readTrajFromCSV(2, "acc");
//
//  // Load the urdf model
//  Model model;
//  pinocchio::urdf::buildModel(urdf_filename,model);
//
//  // Create data required by the algorithms
//  Data data(model);
//
//  // Sample a random configuration
//  Eigen::VectorXd q = qTraj.col(0);
//  Eigen::VectorXd dq = dqTraj.col(0);
//  Eigen::VectorXd ddq = ddqTraj.col(0);
//  ScalarMatrix dV_dq = ScalarMatrix::Zero(6, model.nv);
//  ScalarMatrix dV_ddq = ScalarMatrix::Zero(6, model.nv);
//  // Perform the forward kinematics over the kinematic tree
//  forwardKinematics(model,data,q);
//  updateFramePlacements(model, data);
//  computeForwardKinematicsDerivatives(model, data, q, dq, ddq);
//
//  int IMU1 = model.getFrameId("IMU1_link");
//  pinocchio::getFrameVelocityDerivatives(model, data, IMU1, LOCAL, dV_dq, dV_ddq);
//
////  std::cout << "dV/dq" << dV_dq << std::endl;
////  std::cout << "dV/ddq" << dV_ddq << std::endl;
////
//  ADModel pinADModel = model.cast<ADScalar>();
//  ADData pinADData(pinADModel);
//  ADVector X_vel = ADMatrix::Zero(13*2,1);
//  X_vel.head(13) = randomConfiguration(pinADModel);
//
//  X_vel.segment(13, 13)= ADTangentVectorType::Random(13);
//  std::cout << "X_vel : " << X_vel.transpose() << std::endl;
////  X_vel.tail(13)= ADTangentVectorType::Random(13);
////  			std::cout << "Independent " << std::endl;
//  	        CppAD::Independent(X_vel);
//
//  	        ADVector ad_q(13), ad_dq(13);
////  	        std::cout << "x_to_q_dq " << std::endl;
//  	        //x_to_q_dq<ADVector>(X_vel, ad_q, ad_dq);
//  	        ad_q = X_vel.head(13);
//  	        ad_dq = X_vel.segment(13,13);
//  	        std::cout << "Forward kinematics " << std::endl;
//  	        pin::forwardKinematics(pinADModel, pinADData, ad_q, ad_dq);
//  	        pin::updateFramePlacements(pinADModel, pinADData);
////  	        std::cout << "GetFrameVel " << std::endl;
//  	        ADVector gyr(3); gyr = getFrameVelocity(pinADModel, pinADData, IMU1).angular();
//
////  	        std::cout << "Make dependent " << std::endl;
//  	        /**Generate AD function and stop recording **/
//  	        //this->gyrCostFun->Dependent(X_vel, res);
////  	        std::cout << "Made dependent " << std::endl;
//  	        ADFun<Scalar> fkine_gyr(X_vel,gyr);
//  	ScalarVector X_vel2 = ScalarMatrix::Zero(13*2,1);
//  	X_vel2.head(13) = q;
//  	X_vel2.tail(13) = dq;
//  	std::cout << "X_vel 2 : " << X_vel2.transpose() << std::endl;
//  	//X_vel2.tail(13) = dq;
//  	ScalarVector es_meas = fkine_gyr.Forward(0, X_vel2);
//	std::cout << "est vel : " << es_meas.transpose() << std::endl;
//	std::cout << "ref vel : " << getFrameVelocity(model, data, IMU1).angular().transpose() << std::endl;
//  	ScalarVector ad_jac = fkine_gyr.Jacobian(X_vel2);
//  	ScalarMatrix est_jac = ScalarMatrix::Zero(3, 13);
//  	for (int i =0; i<3; i++){
//  		est_jac.row(i) = ad_jac.segment(i*13,13).transpose();
//  	}
//  	std::cout << "dV/dq : " << dV_dq << std::endl;
//  	std::cout << "ad_jac.transpose : " << ad_jac.transpose() << std::endl;
//  	std::cout << "dV/dq : " << est_jac << std::endl;
//  	//std::cout << "est vel : " << es_meas.transpose() << std::endl;
//  	//std::cout << "ref vel : " << getFrameVelocity(model, data, IMU1).angular().transpose() << std::endl;
//
////  // Get the position of the elbow joint
//
////  Eigen::VectorXd elbow_position = data.oMf[elbowJointID].translation();
//}
//
//
//#include "pinocchio/parsers/urdf.hpp"
//#include "pinocchio/algorithm/joint-configuration.hpp"
//#include "pinocchio/algorithm/kinematics.hpp"
//
//int main(int argc, char ** argv)
//{
//  using namespace pinocchio;
//  //Setting the joint models
//  pinocchio::JointModelRevoluteTpl<double, 0, 0> revolute_jointX;
//  pinocchio::JointModelRevoluteTpl<double, 0, 1> revolute_jointY;
//  pinocchio::JointModelRevoluteTpl<double, 0, 2> revolute_jointZ;
//
//  //4x4 placement matrix containing Orientation and position
//  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(4,4);
//
//  //Set the placement of each joint according to P
//  pinocchio::SE3Tpl<double, 0> joint1_placement(P);
//  pinocchio::SE3Tpl<double, 0> joint2_placement(P);
//  pinocchio::SE3Tpl<double, 0> joint3_placement(P);
//
//  // Instanciate generic model
//  Model model;
//
//  //Add joints tree to the model
//  model.addJoint(0, revolute_jointX, joint1_placement, "joint1");
//  model.addJoint(model.getFrameId("joint1"), revolute_jointY, joint2_placement, "joint2");
//  model.addJoint(model.getFrameId("joint2"), revolute_jointZ, joint3_placement, "joint3");
//
//  //Append a body to last joint
//  Eigen::MatrixXd inertiaMat = Eigen::MatrixXd::Identity(6,6);
//  pinocchio::InertiaTpl<double, 0> body_inertia(inertiaMat);
//  pinocchio::SE3Tpl<double, 0> body_placement(P);
//  model.appendBodyToJoint(model.getFrameId("joint1"), body_inertia, body_placement);
//
//  // Create data required by the algorithms
//  Data data(model);
//
//  // Sample a random configuration
//  Eigen::VectorXd q = Eigen::MatrixXd::Zero(13,1);
//
//  // Perform the forward kinematics over the kinematic tree
//  forwardKinematics(model,data,q);
//  updateFramePlacements(model, data);
//
//  // Get the position of the elbow joint
//  int joint3_ID = model.getFrameId("joint3");
//  Eigen::VectorXd joint3_position = data.oMf[joint3_ID].translation();
//}
