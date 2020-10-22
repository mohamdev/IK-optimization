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
Sensor::Sensor(sensorType const & typeSensor, dataType const & typeData){
	this->setSensType(typeSensor);
	this->setDataType(typeData);
	if (typeSensor == GYR || typeSensor == ACC || typeSensor == POS){
		this->value == ScalarMatrix::Zero(3,1);
	}else this->value == ScalarMatrix::Zero(4,1);         //Initialize value at the right dimensions
};

Sensor::~Sensor(){

};

ScalarVector Sensor::getValue() const{
	return this->value;
}
void Sensor::setValue(ScalarVector const & newValue){
	this->value = newValue;
}

std::string Sensor::getID() const{
	return this->ID;
}
void Sensor::setID(std::string newID){
	this->ID = newID;
}

sensorType Sensor::getSensType() const{
	return this->typeSens;
}
void Sensor::setSensType(sensorType typeSensor){
	this->typeSens = typeSensor;
}

dataType Sensor::getDataType() const{
	return this->typeDat;
}
void Sensor::setDataType(dataType typeData){
	this->typeDat = typeData;
}


/* -------------- CLASS Vimu IMPLEMENTATION ------------*/
Vimu::Vimu(){
	this->ID = 0;
}
Vimu::Vimu(int const & nb_meas_variables, Model const & model, std::string ID){
	this->meas = ScalarVector::Zero(nb_meas_variables,1);
	this->ID = model.getFrameId(ID);
	Eigen::Vector3d gravity(0,0,9.806);
	this->g = gravity;
}
Vimu::~Vimu(){}

void Vimu::setMeas(Model * pinModel, Data * pinData, JointStates const & dataLimb){
    pinocchio::forwardKinematics(*pinModel, *pinData, dataLimb.q, dataLimb.dq, dataLimb.ddq);
    updateFramePlacements(*pinModel, *pinData);
    this->meas.head(3) = pinData->oMf[this->ID].translation(); //Position
    this->meas.segment(3,3) = getFrameVelocity(*pinModel, *pinData, this->ID).angular(); //Velocity
    this->R = pinData->oMf[this->ID].rotation(); //Rotation
    this->meas.segment(6,3) = getFrameAcceleration(*pinModel, *pinData, this->ID).angular() + this->R.transpose()*this->g; //Acceleration
    this->meas.tail(4) = rot2quat(R); //Quaternion
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

//void Limb::refreshMeasurement


