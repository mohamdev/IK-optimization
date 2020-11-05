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
Sensor::Sensor(Model const & model, int const & nb_sensor_var, int const & nb_state_variables, std::string ID){
	this->nb_meas_var = nb_sensor_var;
	this->nb_states = nb_state_variables;
//	this->posCostFun = std::make_shared<ADFun<Scalar>>();
//	this->gyrCostFun = std::make_shared<ADFun<Scalar>>();
//	this->posCostFun = std::make_shared<ADFun<Scalar>>();
//	this->quatCostFun = std::make_shared<ADFun<Scalar>>();
//	this->quatCostFun = nullptr;
//	this->posCostFun = nullptr;
//	this->gyrCostFun = nullptr;
//	this->accCostFun = nullptr;
	this->residual = 0;
	this->meas = ScalarMatrix::Zero(nb_sensor_var, 1);
	this->refMeas = ScalarMatrix::Zero(nb_sensor_var, 1);
	this->jacPos = ScalarMatrix::Zero(nb_state_variables*3, 1);
	this->jacGyr = ScalarMatrix::Zero(nb_state_variables*3, 1);
	this->jacAcc = ScalarMatrix::Zero(nb_state_variables*3, 1);
	this->jacQuat = ScalarMatrix::Zero(nb_state_variables*3, 1);
	this->resJac = ScalarMatrix::Zero((nb_state_variables*3)*4,1);
	this->ID = model.getFrameId(ID);
	Eigen::Vector3d gravity(0,0,9.806);
	this->g = gravity;
};

Sensor::~Sensor(){

};

int Sensor::getSize() const{
	return this->nb_meas_var;
}

ScalarVector Sensor::getResidualsJacobian() const {
	return this->resJac.tail(84);
}
ScalarVector Sensor::getMeas(dataType const & typeData) const{
	if (typeData == REF){
		return this->refMeas;
	}else return this->meas;

}
void Sensor::setMeas(ScalarVector const & newValue){
	if(newValue.rows() == this->meas.rows()){
		this->meas = newValue;
	}else cout << "Can't change measurement, wrong ScalarVector input size in setMeas() input" << endl;

}
void Sensor::setMeas(Model const & pinModel, Data & pinData, JointStates const & dataLimb){
	if(dataLimb.typeDat == REF){
	    pinocchio::forwardKinematics(pinModel, pinData, dataLimb.q, dataLimb.dq, dataLimb.ddq);
	    updateFramePlacements(pinModel,pinData);
	    this->refMeas.head(3) = pinData.oMf[this->ID].translation(); //Position
	    this->refMeas.segment(3,3) = getFrameVelocity(pinModel, pinData, this->ID).angular(); //Velocity
	    this->R = pinData.oMf[this->ID].rotation(); //Rotation
	    this->refMeas.segment(6,3) = getFrameAcceleration(pinModel, pinData, this->ID).angular() + this->R.transpose()*this->g; //Acceleration
	    this->refMeas.tail(4) = rot2quat(R); //Quaternion
	}else if(dataLimb.typeDat == EST){
	    pinocchio::forwardKinematics(pinModel, pinData, dataLimb.q, dataLimb.dq, dataLimb.ddq);
	    updateFramePlacements(pinModel,pinData);
	    this->meas.head(3) = pinData.oMf[this->ID].translation(); //Position
	    this->meas.segment(3,3) = getFrameVelocity(pinModel, pinData, this->ID).angular(); //Velocity
	    this->R = pinData.oMf[this->ID].rotation(); //Rotation
	    this->meas.segment(6,3) = getFrameAcceleration(pinModel, pinData, this->ID).angular() + this->R.transpose()*this->g; //Acceleration
	    this->meas.tail(4) = rot2quat(R); //Quaternion
	}else std::cout << "dataLimb typeDat not defined" << std::endl;

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

int Sensor::getNbMeasVar(){
	return this->nb_meas_var;
}

void Sensor::setPosADFun(ADModel const & pinADModel, ADData & pinADData){
	        /**Set an AD configuration ad_q **/

	        ADConfigVectorType ad_q(pinADModel.nv);
	        ADConfigVectorType & X = ad_q;
	        ADVector refPos = ADMatrix::Zero(3,1);
	        CppAD::Independent(X, refPos);//, refPos);

	        pin::forwardKinematics(pinADModel, pinADData, ad_q);
	        pin::updateFramePlacements(pinADModel, pinADData);
	        ADVector pos(3); pos = pinADData.oMf[this->ID].translation();
	        ADVector res(1);
	        res(0) = (pos - refPos).squaredNorm();
	        /**Generate AD function and stop recording **/
	        //this->posCostFun->Dependent(X, res);
	        ADFun<Scalar> posCost(X, res);
	        this->posCostFun = std::move(posCost);

}
//
void Sensor::setQuatADFun(ADModel const & pinADModel, ADData & pinADData){
	        /**Set an AD configuration ad_q **/
			ADConfigVectorType ad_q(pinADModel.nv);
			ADConfigVectorType & X = ad_q;
			ADVector refQuat = ADMatrix::Zero(4,1);
	        Independent(X, refQuat);

	        pin::forwardKinematics(pinADModel, pinADData, ad_q);
	        pin::updateFramePlacements(pinADModel, pinADData);
	        ADMatrix R = ADMatrix::Zero(3,3); R = pinADData.oMf[this->ID].rotation();
	        ADVector quat(4); rot2quatAD(R,quat);
	        ADVector res(1);
	        res(0) = (quat - refQuat).squaredNorm();
	        //this->quatCostFun->Dependent(X,res);
	        /**Generate AD function and stop recording **/
	        ADFun<Scalar> fkine_quat(X,res);
	        this->quatCostFun = std::move(fkine_quat);
}
////
void Sensor::setGyrADFun(ADModel const & pinADModel, ADData & pinADData){
	        /**Set an AD configuration ad_q **/

			ADVector X_vel(pinADModel.nv*2);
			ADVector refGyr = ADMatrix::Zero(3,1);
			std::cout << "Independent " << std::endl;
	        CppAD::Independent(X_vel, refGyr);

	        ADVector ad_q(pinADModel.nv), ad_dq(pinADModel.nv);
	        std::cout << "x_to_q_dq " << std::endl;
	        x_to_q_dq<ADVector>(X_vel, ad_q, ad_dq);
	        std::cout << "Forward kinematics " << std::endl;
	        pin::forwardKinematics(pinADModel, pinADData, ad_q, ad_dq);
	        pin::updateFramePlacements(pinADModel, pinADData);
	        std::cout << "GetFrameVel " << std::endl;
	        ADVector gyr(3); gyr = getFrameVelocity(pinADModel, pinADData, this->ID).angular();

	        ADVector res(1);
	        res(0) = (gyr - refGyr).squaredNorm();
	        std::cout << "Make dependent " << std::endl;
	        /**Generate AD function and stop recording **/
	        //this->gyrCostFun->Dependent(X_vel, res);
	        std::cout << "Made dependent " << std::endl;
	        ADFun<Scalar> fkine_gyr(X_vel,res);
	        this->gyrCostFun = std::move(fkine_gyr);
}
//
void Sensor::setAccADFun(ADModel const & pinADModel, ADData & pinADData){
	        /**Set an AD configuration ad_q **/

			ADVector X_acc(pinADModel.nv*3);
			ADVector refAcc = ADMatrix::Zero(3,1);

	        Independent(X_acc, refAcc);
	        ADVector ad_q(pinADModel.nv), ad_dq(pinADModel.nv), ad_ddq(pinADModel.nv);
			ADVector g = ADMatrix::Zero(3,1);
			g(2) = 9.806;
	        x_to_q_dq_ddq<ADVector>(X_acc, ad_q, ad_dq, ad_ddq);
	        pin::forwardKinematics(pinADModel, pinADData, ad_q, ad_dq, ad_ddq);
	        pin::updateFramePlacements(pinADModel, pinADData);
	        ADMatrix R = ADMatrix::Zero(3,3); R = pinADData.oMf[this->ID].rotation();
	        std::cout << "GetFrameAcceleration " << std::endl;
	        ADVector acc(3); acc = getFrameAcceleration(pinADModel, pinADData, this->ID).linear() + R.transpose()*g;
	        ADVector res(1);
	        res(0) = (acc - refAcc).squaredNorm();
	        std::cout << "Make dependent " << std::endl;
	        //this->accCostFun->Dependent(X_acc, res);
	        std::cout << "Made dependent " << std::endl;
	        /**Generate AD function and stop recording **/
	        ADFun<Scalar> fkine_acc(X_acc,res);
	        this->accCostFun = std::move(fkine_acc);
}
//
void Sensor::setADFuns(ADModel const & pinADModel, ADData & pinADData){
	std::cout << "ADFUn pos " << std::endl;
	this->setPosADFun(pinADModel, pinADData);
	std::cout << "ADFUn GYr " << std::endl;
	this->setGyrADFun(pinADModel, pinADData);
	std::cout << "ADFUn Acc " << std::endl;
	this->setAccADFun(pinADModel, pinADData);
	std::cout << "ADFun quat " << std::endl;
	this->setQuatADFun(pinADModel, pinADData);
}
//
void Sensor::setResiduals_and_Jacobian(Model const & pinModel, Data & pinData, JointStates const & refStates, JointStates const & estStates){
	/* This function refreshes the residuals of the sensor.
	 * refreshSensors(REF) and refreshSensors(EST) of the limb to which the sensor belongs
	 *  have to be called before this function
	 * */
	this->setMeas(pinModel, pinData, refStates);
	this->setMeas(pinModel, pinData, estStates);
	ScalarVector refPos = this->refMeas.head(3);
	ScalarVector refGyr = this->refMeas.segment(3,3);
	ScalarVector refAcc = this->refMeas.segment(6,3);
	ScalarVector refQuat = this->refMeas.tail(4);
//    this->posCostFun->new_dynamic(refPos);
//    this->gyrCostFun->new_dynamic(refGyr);
//    this->accCostFun->new_dynamic(refAcc);
//    this->quatCostFun->new_dynamic(refQuat);
    this->posCostFun.new_dynamic(std::move(refPos));
    this->gyrCostFun.new_dynamic(std::move(refGyr));
    this->accCostFun.new_dynamic(std::move(refAcc));
    this->quatCostFun.new_dynamic(std::move(refQuat));

    ScalarVector Xq = ScalarMatrix::Zero(estStates.q.rows(),1);
    ScalarVector Xq_dq = ScalarMatrix::Zero(estStates.q.rows()*2,1);
    ScalarVector Xq_dq_ddq = ScalarMatrix::Zero(estStates.q.rows()*3,1);

    q_dq_to_x(Xq_dq, estStates.q, estStates.dq);
    q_dq_ddq_to_x(Xq_dq_ddq, estStates.q, estStates.dq, estStates.ddq);

//    this->residual = (this->posCostFun->Forward(0,Xq) + this->gyrCostFun->Forward(0,Xq_dq) + this->accCostFun->Forward(0,Xq_dq_ddq) + this->quatCostFun->Forward(0,Xq))(0);
//
//    this->jacPos.head(13) = this->posCostFun->Jacobian(Xq);
//    this->jacGyr.head(26) = this->gyrCostFun->Jacobian(Xq_dq);
//    this->jacAcc = this->accCostFun->Jacobian(Xq_dq_ddq);
//    this->jacQuat.head(13) = this->quatCostFun->Jacobian(Xq);
    this->residual = (this->posCostFun.Forward(0,Xq)
    				+ this->gyrCostFun.Forward(0,Xq_dq)
					+ this->accCostFun.Forward(0,Xq_dq_ddq)
					+ this->quatCostFun.Forward(0,Xq))(0);

    this->jacPos.head(13) = this->posCostFun.Jacobian(Xq);
    this->jacGyr.head(26) = this->gyrCostFun.Jacobian(std::move(Xq_dq));
    this->jacAcc = this->accCostFun.Jacobian(std::move(Xq_dq_ddq));
    this->jacQuat.head(13) = this->quatCostFun.Jacobian(std::move(Xq));
    this->resJac.head(39) = this->jacPos;
    this->resJac.segment(39,39) = this->jacGyr;
    this->resJac.segment(39*2,39) = this->jacAcc;
    this->resJac.segment(39*3, 39) = this->jacQuat;
}




/* -------------- CLASS Limb IMPLEMENTATION -------------*/

Limb::Limb(string const & urdf_filename, int const & nb_state_variables, int const & nb_measurement_variables, int const & nb_sensors){

    pin::urdf::buildModel(urdf_filename, this->pinModel);
    this->nb_sensors = nb_sensors;
    this->pinData = Data(this->pinModel);
    this->setDataDimensions(nb_state_variables, nb_measurement_variables);
    this->CppADModel = this->pinModel.cast<ADScalar>();
    ADData newDat(this->CppADModel);
    this->CppADData = newDat;
}
Limb::~Limb(){

};

void Limb::setDataDimensions(int const & nb_state_variables, int const & nb_meas_variables){
    this->nb_meas = nb_meas_variables;
    this->nb_states = nb_state_variables;
	this->estState.typeDat = EST;
	this->estState.q = ScalarMatrix::Zero(nb_state_variables,1);
	this->estState.dq = ScalarMatrix::Zero(nb_state_variables,1);
	this->estState.ddq = ScalarMatrix::Zero(nb_state_variables,1);
	this->estMeas = ScalarMatrix::Zero(nb_meas_variables,1);
	this->limbResJacobian = ScalarMatrix::Zero(156*3-216,1);
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
	std::shared_ptr<Sensor> newSensorPtr = std::make_shared<Sensor>(this->pinModel, nb_sensor_var, this->nb_states, ID);
	std::cout << "Setting AD Funs " << std::endl;
	newSensorPtr->setADFuns(this->CppADModel, this->CppADData);
	std::cout << "Pushing back sensor " << std::endl;
	this->sensors.push_back(std::move(newSensorPtr));

}

void Limb::refreshSensors(dataType const & typeData){
	/*This function refreshes the measurement of each Sensor object, either REF or EST, using pinModel and pinData */
//	if(typeData == EST){
//		for (unsigned int i =0; i<this->sensors.size(); i++)
//		{
//			this->sensors[i].setMeas(this->pinModel, this->pinData, this->estState);
//		}
//		this->refreshLimbMeas(typeData);
//	}else if (typeData == REF){
//		for (unsigned int i =0; i<this->sensors.size(); i++)
//		{
//			this->sensors[i].setMeas(this->pinModel, this->pinData, this->refState);
//		}
//		this->refreshLimbMeas(typeData);
//	}
	if(typeData == EST){
		for (unsigned int i =0; i<this->sensors.size(); i++)
		{
			this->sensors[i]->setMeas(this->pinModel, this->pinData, this->estState);
		}
		this->refreshLimbMeas(typeData);
	}else if (typeData == REF){
		for (unsigned int i =0; i<this->sensors.size(); i++)
		{
			this->sensors[i]->setMeas(this->pinModel, this->pinData, this->refState);
		}
		this->refreshLimbMeas(typeData);
	}
}




void Limb::refreshLimbMeas(dataType const & typeData){
	/*This function refreshes the measurement of the measurement vectors of the Limb object
	 * getting them directly from each Sensor object
	 * A call of refreshSensors() is needed before this method */
//	if (typeData == EST)
//	{
//		ScalarVector newMeas = ScalarMatrix::Zero(this->estMeas.rows(), 1);
//		for (unsigned int i = 0; i<this->sensors.size(); i++)
//		{
//			this->estMeas.segment(i*this->sensors[i].getNbMeasVar(), this->sensors[i].getNbMeasVar()) = this->sensors[i].getMeas(EST);
//		}
//		this->estTraj.measTraj.push_back(this->estMeas);
//	}else if (typeData == REF)
//	{
//		ScalarVector newMeas = ScalarMatrix::Zero(this->refMeas.rows(), 1);
//		for (unsigned int i = 0; i<this->sensors.size(); i++)
//		{
//			this->refMeas.segment(i*this->sensors[i].getNbMeasVar(), this->sensors[i].getNbMeasVar()) = this->sensors[i].getMeas(REF);
//		}
//		this->refTraj.measTraj.push_back(this->refMeas);
//	}
	if (typeData == EST)
	{
		ScalarVector newMeas = ScalarMatrix::Zero(this->estMeas.rows(), 1);
		for (unsigned int i = 0; i<this->sensors.size(); i++)
		{
			this->estMeas.segment(i*this->sensors[i]->getNbMeasVar(), this->sensors[i]->getNbMeasVar()) = this->sensors[i]->getMeas(EST);
		}
		this->estTraj.measTraj.push_back(this->estMeas);
	}else if (typeData == REF)
	{
		ScalarVector newMeas = ScalarMatrix::Zero(this->refMeas.rows(), 1);
		for (unsigned int i = 0; i<this->sensors.size(); i++)
		{
			this->refMeas.segment(i*this->sensors[i]->getNbMeasVar(), this->sensors[i]->getNbMeasVar()) = this->sensors[i]->getMeas(REF);
		}
		this->refTraj.measTraj.push_back(this->refMeas);
	}
}

void Limb::setJointPos(ScalarVector const & newJointPos, dataType const & typeData){
	if (typeData == REF){
		this->refState.q = newJointPos;
		this->refTraj.qTraj.push_back(newJointPos);
	}else if (typeData == EST){
		this->estState.q = newJointPos;
		this->estTraj.qTraj.push_back(newJointPos);
	}
}

ScalarVector Limb::getJointPos(dataType const & typeData) const{
	if (typeData == REF){
		return this->refState.q;
	}else if (typeData == EST) {
		return this->estState.q;
	}else return this->estState.q;
}

void Limb::setLimb_res_Jacobian(){

	/* This function refreshes the residuals of each sensor, and refreshes the limbResJacobian
	 * refreshSensors(REF) and refreshSensors(EST) have to be called before this function
	 *
	 * */
//	for (int i = 0; i<3; i++){
//		this->sensors[i].setResiduals_and_Jacobian(this->pinModel, this->pinData, this->refState, this->estState);
//		this->limbResJacobian.segment(i*84, 84) = this->sensors[i].getResidualsJacobian();
//	}
	for (int i = 0; i<3; i++){
		this->sensors[i]->setResiduals_and_Jacobian(this->pinModel, this->pinData, this->refState, this->estState);
		this->limbResJacobian.segment(i*84, 84) = this->sensors[i]->getResidualsJacobian();
	}
}

ScalarVector Limb::getLimb_res_Jacobian() const{
		return this->limbResJacobian;
}


