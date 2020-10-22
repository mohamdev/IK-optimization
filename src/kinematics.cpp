/*
 * kinematics.cpp
 *
 *  Created on: 21 Oct 2020
 *      Author: aladinedev2
 */

#include "kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include <iostream>

/* -------------- CLASS Sensor IMPLEMENTATION -------------*/
Sensor::Sensor(Model const & model, dataType const & typeData, int const & nb_meas_variables, std::string ID){
	this->setDataType(typeData);
	this->meas = ScalarVector::Zero(nb_meas_variables,1);
	this->ID = model.getFrameId(ID);
	Eigen::Vector3d gravity(0,0,9.806);
	this->g = gravity;
};

Sensor::~Sensor(){

};

ScalarVector Sensor::getMeas() const{
	return this->meas;
}
void Sensor::setMeas(ScalarVector const & newValue){
	if(newValue.rows() == this->meas.rows()){
		this->meas = newValue;
	}else cout << "Can't change measurement, wrong ScalarVector input size in setMeas() input" << endl;

}
void Sensor::setMeas(Model * pinModel, Data * pinData, JointStates const & dataLimb){
    pinocchio::forwardKinematics(*pinModel, *pinData, dataLimb.q, dataLimb.dq, dataLimb.ddq);
    updateFramePlacements(*pinModel, *pinData);
    this->meas.head(3) = pinData->oMf[this->ID].translation(); //Position
    this->meas.segment(3,3) = getFrameVelocity(*pinModel, *pinData, this->ID).angular(); //Velocity
    this->R = pinData->oMf[this->ID].rotation(); //Rotation
    this->meas.segment(6,3) = getFrameAcceleration(*pinModel, *pinData, this->ID).angular() + this->R.transpose()*this->g; //Acceleration
    this->meas.tail(4) = rot2quat(R); //Quaternion
}

int Sensor::getID() const{
	return this->ID;
}

dataType Sensor::getDataType() const{
	return this->typeDat;
}
void Sensor::setDataType(dataType typeData){
	this->typeDat = typeData;
}


/* -------------- CLASS Limb IMPLEMENTATION -------------*/

Limb::Limb(string const & urdf_filename, int const & nb_state_variables, int const & nb_measurement_variables){

    pinocchio::urdf::buildModel(urdf_filename, this->pinModel);
    this->pinData = Data(this->pinModel);
    this->initializeLimbData(nb_state_variables, nb_measurement_variables);
}

void Limb::initializeLimbData(int const & nb_state_variables, int const & nb_meas_variables){
	this->estState.typeDat = EST;
	this->estState.q = ScalarMatrix::Zero(nb_state_variables,1);
	this->estState.dq = ScalarMatrix::Zero(nb_state_variables,1);
	this->estState.ddq = ScalarMatrix::Zero(nb_state_variables,1);
	this->estMeas = ScalarMatrix::Zero(nb_meas_variables,1);

	this->refState.typeDat = REF;
	this->refState.q = ScalarMatrix::Zero(nb_state_variables,1);
	this->refState.dq = ScalarMatrix::Zero(nb_state_variables,1);
	this->refState.ddq = ScalarMatrix::Zero(nb_state_variables,1);
	this->refMeas = ScalarMatrix::Zero(nb_meas_variables,1);
}

void Limb::addSensor(int const & nb_sensor_variables, string ID){
	Sensor newEstSensor(this->pinModel, EST, nb_sensor_variables, ID);
	Sensor newRefSensor(this->pinModel, REF, nb_sensor_variables, ID);
	this->estSensors.push_back(newEstSensor);
	this->refSensors.push_back(newRefSensor);
}

//void Limb::refreshMeasurement


