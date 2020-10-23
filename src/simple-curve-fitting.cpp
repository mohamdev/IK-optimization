/*
 * simple-curve-fitting.cpp
 *
 *  Created on: 23 Oct 2020
 *      Author: aladinedev2
 */


//
#include "kinematics.hpp"
#include "polynom.h"
#include "matplotlibcpp.h"
#define dT 0.01
namespace plt = matplotlibcpp;

int main(int argc, char ** argv)
{
  using namespace pinocchio;

	Polynom pol1(5,1);
	//pol1.setDim(2);
	pol1.generateRandTraj(dT, 0.1, 500);
	ScalarMatrix traj;
	traj = pol1.getTraj();
  const std::string urdf_filename = "/home/aladinedev2/Desktop/pinocchio-cpp/urdf/human_arm_p.urdf";
  int nb_vimu = 1;
  int nb_joints = 13;

  // Load the urdf model
  Limb arm = Limb(urdf_filename, nb_joints, nb_vimu*13); //Instantiate an arm from urdf_filename, nb_states = 13 and nb_measurements = 3
  arm.addSensor(13, "elbow_joint_q10");

  ScalarVector q = ScalarMatrix::Zero(13,1);
  //q(5) = traj(0,0);
  for (int i =0; i<traj.cols(); i++){
	 q(6) = traj(0,i);

	  arm.setJointPos(q, REF);
	  arm.refreshAllSensors(REF);

	  arm.timesample.push_back(i);
	  arm.t.push_back(i*dT);
	  //arm.refreshAllSensors(EST);
  }
  vector<vector<Scalar>> qtraj;
  eigen2vector(arm.refTraj.qTraj, qtraj);

  vector<vector<Scalar>> measTraj;
  eigen2vector(arm.refTraj.measTraj, measTraj);

  plt::subplot(4,1,1);
  plt::named_plot("Angle q",arm.t,qtraj[6]);
  plt::legend();

  plt::subplot(4,1,2);
  plt::named_plot("Sensor pos X", arm.t,measTraj[0]);
  plt::legend();

  plt::subplot(4,1,3);
  plt::named_plot("Sensor pos Y", arm.t,measTraj[1]);
  plt::legend();

  plt::subplot(4,1,4);
  plt::named_plot("Sensor pos Z", arm.t,measTraj[2]);
  plt::legend();

  plt::show();

  cout << "finish" << endl;


}

