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
Sensor::Sensor(Model const & model, int const & nb_meas_variables, dataType const & typeData, std::string ID){
	this->setDataType(typeData);
	this->nb_meas_var = nb_meas_variables;
	this->meas = ScalarMatrix::Zero(nb_meas_variables, 1);
	this->ID = model.getFrameId(ID);
	Eigen::Vector3d gravity(0,0,9.806);
	this->g = gravity;

};

Sensor::~Sensor(){

};

int Sensor::getSize() const{
	return this->nb_meas_var;
}

ScalarVector Sensor::getMeas() const{
	return this->meas;
}
void Sensor::setMeas(ScalarVector const & newValue){
	if(newValue.rows() == this->meas.rows()){
		this->meas = newValue;
	}else cout << "Can't change measurement, wrong ScalarVector input size in setMeas() input" << endl;

}
void Sensor::setMeas(Model const & pinModel, Data & pinData, JointStates const & dataLimb){
    pinocchio::forwardKinematics(pinModel, pinData, dataLimb.q, dataLimb.dq, dataLimb.ddq);
    updateFramePlacements(pinModel,pinData);
    this->meas.head(3) = pinData.oMf[this->ID].translation(); //Position
    this->meas.segment(3,3) = getFrameVelocity(pinModel, pinData, this->ID).angular(); //Velocity
    this->R = pinData.oMf[this->ID].rotation(); //Rotation
    this->meas.segment(6,3) = getFrameAcceleration(pinModel, pinData, this->ID).angular() + this->R.transpose()*this->g; //Acceleration
    this->meas.tail(4) = rot2quat(R); //Quaternion
}

void Sensor::setMeas(Model const & pinModel, Data & pinData, ScalarVector const & q){
    pin::forwardKinematics(pinModel, pinData, q);
    updateFramePlacements(pinModel,pinData);
    this->meas.head(3) = pinData.oMf[this->ID].translation(); //Position
    this->meas.segment(3,3) = getFrameVelocity(pinModel, pinData, this->ID).angular(); //Velocity
    this->R = pinData.oMf[this->ID].rotation(); //Rotation
    this->meas.segment(6,3) = getFrameAcceleration(pinModel, pinData, this->ID).angular() + this->R.transpose()*this->g; //Acceleration
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

int Sensor::getNbMeasVar(){
	return this->nb_meas_var;
}

void Sensor::setPosADFun(ADModel const & pinADModel, ADData & pinADData){
	        /**Set an AD configuration ad_q **/

	        ADConfigVectorType ad_q(pinADModel.nv);
	        ADConfigVectorType & X = ad_q;
	        CppAD::Independent(X);

	        pin::forwardKinematics(pinADModel, pinADData, ad_q);
	        pin::updateFramePlacements(pinADModel, pinADData);
	        ADVector pos(3); pos = pinADData.oMf[this->ID].translation();

	        /**Generate AD function and stop recording **/
	        ADFun<ADScalar> fkine_pos(X,pos);
	        this->posADFun = fkine_pos;
}

void Sensor::setQuatADFun(ADModel const & pinADModel, ADData & pinADData){
	        /**Set an AD configuration ad_q **/
			ADConfigVectorType ad_q(pinADModel.nv);
			ADConfigVectorType & X = ad_q;

	        Independent(X);

	        pin::forwardKinematics(pinADModel, pinADData, ad_q);
	        pin::updateFramePlacements(pinADModel, pinADData);
	        ADMatrix R = ADMatrix::Zero(3,3); R = pinADData.oMf[this->ID].rotation();
	        ADVector quat(4); rot2quatAD(R,quat);

	        /**Generate AD function and stop recording **/
	        ADFun<ADScalar> fkine_quat(X,quat);
	        this->quatADFun = fkine_quat;
}

void Sensor::setGyrADFun(ADModel const & pinADModel, ADData & pinADData){
	        /**Set an AD configuration ad_q **/
			ADConfigVectorType ad_q(pinADModel.nv), ad_dq(pinADModel.nv);
			ADConfigVectorType X_vel(pinADModel.nv*2);

	        CppAD::Independent(X_vel);

	        x_to_q_dq(X_vel, ad_q, ad_dq);
	        pin::forwardKinematics(pinADModel, pinADData, ad_q, ad_dq);
	        pin::updateFramePlacements(pinADModel, pinADData);
	        ADVector gyr(3); gyr = getFrameVelocity(pinADModel, pinADData, this->ID).angular();

	        /**Generate AD function and stop recording **/
	        ADFun<ADScalar> fkine_gyr(X_vel,gyr);
	        this->gyrADFun = fkine_gyr;
}

void Sensor::setAccADFun(ADModel const & pinADModel, ADData & pinADData){
	        /**Set an AD configuration ad_q **/
			ADConfigVectorType ad_q(pinADModel.nv), ad_dq(pinADModel.nv), ad_ddq(pinADModel.nv);
			ADConfigVectorType X_acc(pinADModel.nv*3);

	        Independent(X_acc);

	        x_to_q_dq_ddq(X_acc, ad_q, ad_dq, ad_ddq);
	        pin::forwardKinematics(pinADModel, pinADData, ad_q, ad_dq, ad_ddq);
	        pin::updateFramePlacements(pinADModel, pinADData);
	        ADMatrix R = ADMatrix::Zero(3,3); R = pinADData.oMf[this->ID].rotation();
	        ADVector acc(3); acc = getFrameAcceleration(pinADModel, pinADData, this->ID).linear() + R.transpose()*this->g;

	        /**Generate AD function and stop recording **/
	        ADFun<ADScalar> fkine_acc(X_acc,acc);
	        this->accADFun = fkine_acc;
}
/* -------------- CLASS Limb IMPLEMENTATION -------------*/

Limb::Limb(string const & urdf_filename, int const & nb_state_variables, int const & nb_measurement_variables){

    pin::urdf::buildModel(urdf_filename, this->pinModel);
    this->pinData = Data(this->pinModel);
    this->setDataDimensions(nb_state_variables, nb_measurement_variables);
    this->CppADModel = this->pinModel.cast<ADScalar>();
    this->CppADData(this->CppADModel);
}
Limb::~Limb(){

};

void Limb::setDataDimensions(int const & nb_state_variables, int const & nb_meas_variables){
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

ScalarVector Limb::getMeas(dataType const & typeData) const{
	if (typeData == REF){
		return this->refMeas;
	}else return this->estMeas;

}

void Limb::addSensor(int const & nb_sensor_var, string ID){
	Sensor newEstSensor(this->pinModel, nb_sensor_var, EST, ID);
	Sensor newRefSensor(this->pinModel, nb_sensor_var, REF, ID);
	this->estSensors.push_back(newEstSensor);
	this->refSensors.push_back(newRefSensor);
}

void Limb::refreshAllSensors(dataType const & typeData){
	if(typeData == EST){
		for (unsigned int i =0; i<this->estSensors.size(); i++)
		{
			this->estSensors[i].setMeas(this->pinModel, this->pinData, this->estState);
		}
		this->refreshMeasVector(typeData);
	}else if (typeData == REF){
		for (unsigned int i =0; i<this->refSensors.size(); i++)
		{
			this->refSensors[i].setMeas(this->pinModel, this->pinData, this->refState);
		}
		this->refreshMeasVector(typeData);
	}
}

void Limb::refreshMeasVector(dataType const & typeData){
	if (typeData == EST)
	{
		ScalarVector newMeas = ScalarMatrix::Zero(this->estMeas.rows(), 1);
		for (unsigned int i = 0; i<this->estSensors.size(); i++)
		{
			this->estMeas.segment(i*this->estSensors[i].getNbMeasVar(), this->estSensors[i].getNbMeasVar()) = this->estSensors[i].getMeas();
		}
		this->estTraj.measTraj.push_back(this->estMeas);
	}else if (typeData == REF)
	{
		ScalarVector newMeas = ScalarMatrix::Zero(this->refMeas.rows(), 1);
		for (unsigned int i = 0; i<this->refSensors.size(); i++)
		{
			this->refMeas.segment(i*this->refSensors[i].getNbMeasVar(), this->refSensors[i].getNbMeasVar()) = this->refSensors[i].getMeas();
		}
		this->refTraj.measTraj.push_back(this->refMeas);
	}


}

void Limb::setJointPos(ScalarVector const & newJointPos, dataType const & typeData){
	if (typeData == REF){
		this->refState.q = newJointPos;
		this->refTraj.qTraj.push_back(newJointPos);
	}else if (typeData == EST) this->estState.q = newJointPos; this->estTraj.qTraj.push_back(newJointPos);
}
ScalarVector Limb::getJointPos(dataType const & typeData) const{
	if (typeData == REF){
		return this->refState.q;
	}else if (typeData == EST) {
		return this->estState.q;
	}else return this->estState.q;
}



