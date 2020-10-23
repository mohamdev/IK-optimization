/*
 * simple-forward-kinematics.cpp
 *
 *  Created on: 23 Oct 2020
 *      Author: aladinedev2
 */

//#include "kinematics.hpp"
//
//int main(int argc, char ** argv)
//{
//  using namespace pinocchio;
//
//
//  const std::string urdf_filename = "/home/aladinedev2/Desktop/Final_URDF/human_arm_dorent.urdf";
//
//  // Load the urdf model
//  Model model;
//  pinocchio::urdf::buildModel(urdf_filename,model);
//  std::cout << "model name: " << model.name << std::endl;
//
//  // Create data required by the algorithms
//  Data data(model);
//
//
//  // Sample a random configuration
//  Eigen::VectorXd q = ScalarMatrix::Zero(13,1);
//  std::cout << "q: " << q.transpose() << std::endl;
//  // Perform the forward kinematics over the kinematic tree
//  forwardKinematics(model,data,q);
//  // Print out the placement of each joint of the kinematic tree
//  for(JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
//    std::cout << std::setw(24) << std::left
//              << model.names[joint_id] << ": "
//              << std::fixed << std::setprecision(2)
//              << data.oMi[joint_id].translation().transpose()
//              << std::endl;
//}

