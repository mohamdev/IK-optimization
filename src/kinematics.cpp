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
//#define Te 0.01




/* -------------- CLASS Sensor IMPLEMENTATION -------------*/

//Ajouter le constructeur par défaut.
Sensor::Sensor(){

};

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
//void Sensor::setMeas(Model const & pinModel, Data & pinData, JointStates const & dataLimb){
//	if(dataLimb.typeDat == REF || dataLimb.typeDat == REFeval){
//	    pinocchio::forwardKinematics(pinModel, pinData, dataLimb.q, dataLimb.dq, dataLimb.ddq);
//	    updateFramePlacements(pinModel,pinData);
//	    this->refMeas.head(3) = pinData.oMf[this->ID].translation(); //Position
//	    this->refMeas.segment(3,3) = getFrameVelocity(pinModel, pinData, this->ID).angular(); //Velocity
//	    this->R = pinData.oMf[this->ID].rotation(); //Rotation
//	    this->refMeas.segment(6,3) = getFrameAcceleration(pinModel, pinData, this->ID).angular() + this->R.transpose()*this->g; //Acceleration
//        ScalarVector quat(4); //rot2quatAD(R,quat);
//        Eigen::Quaternion<Scalar> q;
//        pinocchio::quaternion::assignQuaternion(q, R);
//        pinocchio::quaternion::firstOrderNormalize(q);
//        quat(0) = q.w();
//        quat(1) = q.x();
//        quat(2) = q.y();
//        quat(3) = q.z();
//	    this->refMeas.tail(4) = quat; //Quaternion
//	}else if(dataLimb.typeDat == EST || dataLimb.typeDat == ESTeval){
//	    pinocchio::forwardKinematics(pinModel, pinData, dataLimb.q, dataLimb.dq, dataLimb.ddq);
//	    updateFramePlacements(pinModel,pinData);
//	    this->meas.head(3) = pinData.oMf[this->ID].translation(); //Position
//	    this->meas.segment(3,3) = getFrameVelocity(pinModel, pinData, this->ID).angular(); //Velocity
//	    this->R = pinData.oMf[this->ID].rotation(); //Rotation
//	    this->meas.segment(6,3) = getFrameAcceleration(pinModel, pinData, this->ID).angular() + this->R.transpose()*this->g; //Acceleration
//        ScalarVector quat(4);
//	    Eigen::Quaternion<Scalar> q;
//        pinocchio::quaternion::assignQuaternion(q, R);
//        quat(0) = q.w();
//        quat(1) = q.x();
//        quat(2) = q.y();
//        quat(3) = q.z();
//	    this->meas.tail(4) = quat; //Quaternion
//	}else std::cout << "dataLimb typeDat not defined" << std::endl;
//
//}

void Sensor::setMeas(Model const & pinModel, Data & pinData, JointStates const & dataLimb){
	if(dataLimb.typeDat == REF || dataLimb.typeDat == REFeval){
	    pinocchio::forwardKinematics(pinModel, pinData, dataLimb.q, dataLimb.dq, dataLimb.ddq);
	    updateFramePlacements(pinModel,pinData);
	    this->refMeas.head(3) = pinData.oMf[this->ID].translation(); //Position
	    this->refMeas.segment(3,3) = getFrameVelocity(pinModel, pinData, this->ID).angular(); //Velocity
	    this->R = pinData.oMf[this->ID].rotation(); //Rotation
	    this->refMeas.segment(6,3) = getFrameAcceleration(pinModel, pinData, this->ID).linear() + this->R.transpose()*this->g; //Acceleration
	    this->refMeas.tail(4) = rot2quat_tmpl<Scalar>(R);
	}else if(dataLimb.typeDat == EST || dataLimb.typeDat == ESTeval){
	    pinocchio::forwardKinematics(pinModel, pinData, dataLimb.q, dataLimb.dq, dataLimb.ddq);
	    updateFramePlacements(pinModel,pinData);
	    this->meas.head(3) = pinData.oMf[this->ID].translation(); //Position
	    this->meas.segment(3,3) = getFrameVelocity(pinModel, pinData, this->ID).angular(); //Velocity
	    this->R = pinData.oMf[this->ID].rotation(); //Rotation
	    this->meas.segment(6,3) = getFrameAcceleration(pinModel, pinData, this->ID).linear() + this->R.transpose()*this->g; //Acceleration
	    this->meas.tail(4) = rot2quat_tmpl<Scalar>(R);
	}else std::cout << "dataLimb typeDat not defined" << std::endl;

}

void Sensor::setMeas(Model const & pinModel, Data & pinData, ScalarVector const & q){
    pin::forwardKinematics(pinModel, pinData, q);
    updateFramePlacements(pinModel,pinData);
    this->meas.head(3) = pinData.oMf[this->ID].translation(); //Position
    this->meas.segment(3,3) = getFrameVelocity(pinModel, pinData, this->ID).angular(); //Velocity
    this->R = pinData.oMf[this->ID].rotation(); //Rotation
    this->meas.segment(6,3) = getFrameAcceleration(pinModel, pinData, this->ID).linear() + this->R.transpose()*this->g; //Acceleration
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
		ADVector X_vel = ADMatrix::Zero(13*3,1);
		X_vel.head(13) = randomConfiguration(pinADModel);
		X_vel.segment(13, 13)= ADTangentVectorType::Random(13);
		X_vel.tail(13)= ADTangentVectorType::Random(13);
;
	        ADVector refPos = ADMatrix::Zero(3,1);
	        int ID = this->ID;
	        CppAD::Independent(X_vel, refPos);//, refPos);
	        ADVector q = X_vel.head(13);
	        pin::forwardKinematics(pinADModel, pinADData, q);
	        pin::updateFramePlacements(pinADModel, pinADData);
	        ADVector pos(3); pos = pinADData.oMf[ID].translation();
	        ADVector res(1);
	        res(0) = (pos - refPos).squaredNorm();
	        /**Generate AD function and stop recording **/
	        //this->posCostFun->Dependent(X, res);
	        ADFun<Scalar> posCost(X_vel, res);
	        this->posCostFunAD = posCost.base2ad();
	        this->posCostFun = std::move(posCost);

}

void Sensor::setPosFun(ADModel const & pinADModel, ADData & pinADData){
	        /**Set an AD configuration ad_q **/
		ADVector X_vel = ADMatrix::Zero(13*3,1);
		X_vel.head(13) = randomConfiguration(pinADModel);
		X_vel.segment(13, 13)= ADTangentVectorType::Random(13);
		X_vel.tail(13)= ADTangentVectorType::Random(13);
;
	        ADVector refPos = ADMatrix::Zero(3,1);
	        int ID = this->ID;
	        CppAD::Independent(X_vel, refPos);//, refPos);
	        ADVector q = X_vel.head(13);
	        pin::forwardKinematics(pinADModel, pinADData, q);
	        pin::updateFramePlacements(pinADModel, pinADData);
	        ADVector pos(3); pos = pinADData.oMf[ID].translation();
	        /**Generate AD function and stop recording **/
	        //this->posCostFun->Dependent(X, res);
	        ADFun<Scalar> posCost(X_vel, pos);
	        this->posFun = std::move(posCost);

}
//void Sensor::setQuatADFun(ADModel const & pinADModel, ADData & pinADData){
//	        /**Set an AD configuration ad_q **/
//			ADConfigVectorType X = randomConfiguration(pinADModel);
//
//			//ADConfigVectorType & X = ad_q;
//			int ID = this->ID;
//			ADVector refQuat = ADMatrix::Zero(4,1);
//	        Independent(X, refQuat);
//
//	        pin::forwardKinematics(pinADModel, pinADData, X);
//	        pin::updateFramePlacements(pinADModel, pinADData);
//	        ADMatrix R = ADMatrix::Zero(3,3); R = pinADData.oMf[ID].rotation();
//	        ADVector quat(4); //rot2quatAD(R,quat);
//	        Eigen::Quaternion<ADScalar> q;
//	        pinocchio::quaternion::assignQuaternion(q, R);
//	        quat(0) = q.w();
//	        quat(1) = q.x();
//	        quat(2) = q.y();
//	        quat(3) = q.z();
//	        ADVector res(1);
//	        res(0) = (quat - refQuat).squaredNorm();
//	        //this->quatCostFun->Dependent(X,res);
//	        /**Generate AD function and stop recording **/
//	        ADFun<Scalar> fkine_quat(X,res);
//	        this->quatCostFunAD = fkine_quat.base2ad();
//	        this->quatCostFun = std::move(fkine_quat);
//}
void Sensor::setQuatADFun(ADModel const & pinADModel, ADData & pinADData){
	        /**Set an AD configuration ad_q **/
			ADVector X_vel = ADMatrix::Zero(13*3,1);
			X_vel.head(13) = randomConfiguration(pinADModel);
			X_vel.segment(13, 13)= ADTangentVectorType::Random(13);
			X_vel.tail(13)= ADTangentVectorType::Random(13);

			//ADConfigVectorType & X = ad_q;
			int ID = this->ID;
			ADVector refQuat = ADMatrix::Zero(4,1);
	        Independent(X_vel, refQuat);
	        ADVector q = X_vel.head(13);
	        pin::forwardKinematics(pinADModel, pinADData, q);
	        pin::updateFramePlacements(pinADModel, pinADData);
	        ADMatrix R = ADMatrix::Zero(3,3); R = pinADData.oMf[ID].rotation();
	        ADVector quat(4); //rot2quatAD(R,quat);
	        quat = rot2quat_tmpl<ADScalar>(R);
	        ADVector res(1);
	        res(0) = (quat - refQuat).squaredNorm();
	        //this->quatCostFun->Dependent(X,res);
	        /**Generate AD function and stop recording **/
	        ADFun<Scalar> fkine_quat(X_vel,res);
	        this->quatCostFunAD = fkine_quat.base2ad();
	        this->quatCostFun = std::move(fkine_quat);
}

void Sensor::setQuatFun(ADModel const & pinADModel, ADData & pinADData){
	        /**Set an AD configuration ad_q **/
			ADVector X_vel = ADMatrix::Zero(13*3,1);
			X_vel.head(13) = randomConfiguration(pinADModel);
			X_vel.segment(13, 13)= ADTangentVectorType::Random(13);
			X_vel.tail(13)= ADTangentVectorType::Random(13);

			//ADConfigVectorType & X = ad_q;
			int ID = this->ID;
			ADVector refQuat = ADMatrix::Zero(4,1);
	        Independent(X_vel, refQuat);
	        ADVector q = X_vel.head(13);
	        pin::forwardKinematics(pinADModel, pinADData, q);
	        pin::updateFramePlacements(pinADModel, pinADData);
	        ADMatrix R = ADMatrix::Zero(3,3); R = pinADData.oMf[ID].rotation();
	        ADVector quat(4); //rot2quatAD(R,quat);
	        quat = rot2quat_tmpl<ADScalar>(R);
	        //this->quatCostFun->Dependent(X,res);
	        /**Generate AD function and stop recording **/
	        ADFun<Scalar> fkine_quat(X_vel,quat);
	        this->quatFun = std::move(fkine_quat);
}
////
void Sensor::setGyrADFun(ADModel const & pinADModel, ADData & pinADData){

	        /**Set an AD configuration ad_q **/
			ADVector X_vel = ADMatrix::Zero(13*3,1);
			X_vel.head(13) = randomConfiguration(pinADModel);
			X_vel.segment(13, 13)= ADTangentVectorType::Random(13);
			X_vel.tail(13)= ADTangentVectorType::Random(13);
			ADVector refGyr = ADMatrix::Zero(3,1);
			int ID = this->ID;
			std::cout << "Independent " << std::endl;
	        CppAD::Independent(X_vel, refGyr);

	        ADVector ad_q(13), ad_dq(13);
	        std::cout << "x_to_q_dq " << std::endl;
	        //x_to_q_dq<ADVector>(X_vel, ad_q, ad_dq);
	        ad_q = X_vel.head(13);
	        ad_dq = X_vel.segment(13,13);
	        std::cout << "Forward kinematics " << std::endl;
	        pin::forwardKinematics(pinADModel, pinADData, ad_q);
	        pin::updateFramePlacements(pinADModel, pinADData);
	        std::cout << "GetFrameVel " << std::endl;
	        ADVector gyr(3); gyr = getFrameVelocity(pinADModel, pinADData, ID).angular();

	        ADVector res(1);
	        res(0) = (gyr - refGyr).squaredNorm();
	        std::cout << "Make dependent " << std::endl;
	        /**Generate AD function and stop recording **/
	        //this->gyrCostFun->Dependent(X_vel, res);
	        std::cout << "Made dependent " << std::endl;
	        ADFun<Scalar> fkine_gyr(X_vel,res);
			this->gyrCostFunAD = fkine_gyr.base2ad();
	        this->gyrCostFun = std::move(fkine_gyr);
}

////
void Sensor::setGyrFun(ADModel const & pinADModel, ADData & pinADData){

	        /**Set an AD configuration ad_q **/
			ADVector X_vel = ADMatrix::Zero(13*3,1);
			X_vel.head(13) = randomConfiguration(pinADModel);
			X_vel.segment(13, 13)= ADTangentVectorType::Random(13);
			X_vel.tail(13)= ADTangentVectorType::Random(13);
			ADVector refGyr = ADMatrix::Zero(3,1);
			int ID = this->ID;
			std::cout << "Independent " << std::endl;
	        CppAD::Independent(X_vel, refGyr);

	        ADVector ad_q(13), ad_dq(13);
	        std::cout << "x_to_q_dq " << std::endl;
	        //x_to_q_dq<ADVector>(X_vel, ad_q, ad_dq);
	        ad_q = X_vel.head(13);
	        ad_dq = X_vel.segment(13,13);
	        std::cout << "Forward kinematics " << std::endl;
	        pin::forwardKinematics(pinADModel, pinADData, ad_q);
	        pin::updateFramePlacements(pinADModel, pinADData);
	        std::cout << "GetFrameVel " << std::endl;
	        ADVector gyr(3); gyr = getFrameVelocity(pinADModel, pinADData, ID).angular();

	        std::cout << "Make dependent " << std::endl;
	        /**Generate AD function and stop recording **/
	        //this->gyrCostFun->Dependent(X_vel, res);
	        std::cout << "Made dependent " << std::endl;
	        ADFun<Scalar> fkine_gyr(X_vel,gyr);
	        this->gyrFun = std::move(fkine_gyr);
}

//
void Sensor::setAccADFun(ADModel const & pinADModel, ADData & pinADData){
	        /**Set an AD configuration ad_q **/

			ADVector X_acc = ADMatrix::Zero(pinADModel.nv*3,1);
			ADConfigVectorType q = randomConfiguration(pinADModel);
			ADTangentVectorType dq = ADTangentVectorType::Random(pinADModel.nv);
			ADTangentVectorType ddq = ADTangentVectorType::Random(pinADModel.nv);
			q_dq_ddq_to_x<ADVector>(X_acc, q, dq, ddq);
			ADVector refAcc = ADMatrix::Zero(3,1);
			int ID = this->ID;
	        Independent(X_acc, refAcc);
	        ADVector ad_q(pinADModel.nv), ad_dq(pinADModel.nv), ad_ddq(pinADModel.nv);
			ADVector g = ADMatrix::Zero(3,1);
			g(2) = 9.806;
	        x_to_q_dq_ddq<ADVector>(X_acc, ad_q, ad_dq, ad_ddq);
	        pin::forwardKinematics(pinADModel, pinADData, ad_q, ad_dq, ad_ddq);
	        pin::updateFramePlacements(pinADModel, pinADData);
	        ADMatrix R = ADMatrix::Zero(3,3); R = pinADData.oMf[ID].rotation();
	        std::cout << "GetFrameAcceleration " << std::endl;
	        ADVector acc(3); acc = getFrameAcceleration(pinADModel, pinADData, ID).linear() + R.transpose()*g;
	        ADVector res(1);
	        res(0) = (acc - refAcc).squaredNorm();
	        std::cout << "Make dependent " << std::endl;
	        //this->accCostFun->Dependent(X_acc, res);
	        std::cout << "Made dependent " << std::endl;
	        /**Generate AD function and stop recording **/
	        ADFun<Scalar> fkine_acc(X_acc,res);
	        this->accCostFunAD = fkine_acc.base2ad();
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
Limb::Limb(){
 this->nb_sensors = 3;
 this->nb_states = 13;
 this->nb_meas = 39;
}

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
	if (typeData == REF || typeData == REFeval){
		return this->refMeas;
	}else if(typeData == EST || typeData == ESTeval){
		return this->estMeas;
	}else return this->estMeas;
}

//void Limb::addSensor(int const & nb_sensor_var, string ID){
//	std::shared_ptr<Sensor> newSensorPtr = std::make_shared<Sensor>(this->pinModel, nb_sensor_var, this->nb_states, ID);
//	std::cout << "Setting AD Funs " << std::endl;
//	newSensorPtr->setADFuns(this->CppADModel, this->CppADData);
//	std::cout << "Pushing back sensor " << std::endl;
//	this->sensors.push_back(std::move(newSensorPtr));
//
//}

void Limb::addSensor(int const & nb_sensor_var, string ID){
	shared_ptr<Sensor> newSensorPtr = make_shared<Sensor>(this->pinModel, nb_sensor_var, this->nb_states, ID);
	std::cout << "Setting AD Funs " << std::endl;
	newSensorPtr->setADFuns(this->CppADModel, this->CppADData);
	std::cout << "Pushing back sensor " << std::endl;
	this->sensors.push_back(std::move(newSensorPtr));
//	this->sensors[this->sensors.size()-1].setADFuns(this->CppADModel, this->CppADData);

}

void Limb::refreshSensors(dataType const & typeData){
	/*This function refreshes the measurement of each Sensor object, either REF or EST, using pinModel and pinData */
	if(typeData == EST || typeData == ESTeval){
		for (unsigned int i =0; i<this->sensors.size(); i++)
		{
			this->sensors[i]->setMeas(this->pinModel, this->pinData, this->estState);
		}
		this->refreshLimbMeas(typeData);
	}else if (typeData == REF || typeData == REFeval){
		for (unsigned int i =0; i<this->sensors.size(); i++)
		{
			this->sensors[i]->setMeas(this->pinModel, this->pinData, this->refState);
		}
		this->refreshLimbMeas(typeData);
	}
}

//void Limb::refreshSensors(dataType const & typeData){
//	/*This function refreshes the measurement of each Sensor object, either REF or EST, using pinModel and pinData */
////	if(typeData == EST){
////		for (unsigned int i =0; i<this->sensors.size(); i++)
////		{
////			this->sensors[i].setMeas(this->pinModel, this->pinData, this->estState);
////		}
////		this->refreshLimbMeas(typeData);
////	}else if (typeData == REF){
////		for (unsigned int i =0; i<this->sensors.size(); i++)
////		{
////			this->sensors[i].setMeas(this->pinModel, this->pinData, this->refState);
////		}
////		this->refreshLimbMeas(typeData);
////	}
//	if(typeData == EST || typeData == ESTeval){
//		for (unsigned int i =0; i<this->sensors.size(); i++)
//		{
//			this->sensors[i].setMeas(this->pinModel, this->pinData, this->estState);
//		}
//		this->refreshLimbMeas(typeData);
//	}else if (typeData == REF || typeData == REFeval){
//		for (unsigned int i =0; i<this->sensors.size(); i++)
//		{
//			this->sensors[i].setMeas(this->pinModel, this->pinData, this->refState);
//		}
//		this->refreshLimbMeas(typeData);
//	}
//}

//void Limb::refreshLimbMeas(dataType const & typeData){
//	/*This function refreshes the measurement of the measurement vectors of the Limb object
//	 * getting them directly from each Sensor object
//	 * A call of refreshSensors() is needed before this method */
//	if (typeData == EST)
//	{
//		ScalarVector newMeas = ScalarMatrix::Zero(this->estMeas.rows(), 1);
//		for (unsigned int i = 0; i<this->sensors.size(); i++)
//		{
//			this->estMeas.segment(i*this->sensors[i]->getNbMeasVar(), this->sensors[i]->getNbMeasVar()) = this->sensors[i]->getMeas(EST);
//		}
//		this->estTraj.measTraj.push_back(this->estMeas);
//	}else if (typeData == REF)
//	{
//		ScalarVector newMeas = ScalarMatrix::Zero(this->refMeas.rows(), 1);
//		for (unsigned int i = 0; i<this->sensors.size(); i++)
//		{
//			this->refMeas.segment(i*this->sensors[i]->getNbMeasVar(), this->sensors[i]->getNbMeasVar()) = this->sensors[i]->getMeas(REF);
//		}
//		this->refTraj.measTraj.push_back(this->refMeas);
//	}else if (typeData == REFeval)
//	{
//		ScalarVector newMeas = ScalarMatrix::Zero(this->refMeas.rows(), 1);
//		for (unsigned int i = 0; i<this->sensors.size(); i++)
//		{
//			this->refMeas.segment(i*this->sensors[i]->getNbMeasVar(), this->sensors[i]->getNbMeasVar()) = this->sensors[i]->getMeas(REF);
//		}
//	}else if (typeData == ESTeval)
//	{
//		ScalarVector newMeas = ScalarMatrix::Zero(this->refMeas.rows(), 1);
//		for (unsigned int i = 0; i<this->sensors.size(); i++)
//		{
//			this->estMeas.segment(i*this->sensors[i]->getNbMeasVar(), this->sensors[i]->getNbMeasVar()) = this->sensors[i]->getMeas(EST);
//		}
//	}
//}

void Limb::refreshLimbMeas(dataType const & typeData){

	//ScalarVector posGyrQuat = ScalarMatrix::Zero(this->nb_sensors*3 + this->nb_sensors*3 + this->nb_sensors*3 + this->nb_sensors*4, 1);
	if(typeData == REF)
	{
		for (int i =0; i < 3; i++)
		{
			this->refMeas.segment(i*3, 3) = this->sensors[i]->getMeas(typeData).head(3);
			this->refMeas.segment(9 + i*3, 3) = this->sensors[i]->getMeas(typeData).segment(3,3);
			this->refMeas.segment(18 + i*3, 3) = this->sensors[i]->getMeas(typeData).segment(6,3);
			this->refMeas.segment(27 + i*4, 4) = this->sensors[i]->getMeas(typeData).tail(4);
		}
		this->refTraj.measTraj.push_back(this->refMeas);
	}else if(typeData == EST)
	{
		for (int i =0; i < 3; i++)
		{
			this->estMeas.segment(i*3, 3) = this->sensors[i]->getMeas(typeData).head(3);
			this->estMeas.segment(9 + i*3, 3) = this->sensors[i]->getMeas(typeData).segment(3,3);
			this->estMeas.segment(18 + i*3, 3) = this->sensors[i]->getMeas(typeData).segment(6,3);
			this->estMeas.segment(27 + i*4, 4) = this->sensors[i]->getMeas(typeData).tail(4);
		}
		this->estTraj.measTraj.push_back(this->estMeas);
	}else if(typeData == ESTeval)
	{
			for (int i =0; i < 3; i++)
			{
				this->estMeas.segment(i*3, 3) = this->sensors[i]->getMeas(typeData).head(3);
				this->estMeas.segment(9 + i*3, 3) = this->sensors[i]->getMeas(typeData).segment(3,3);
				this->estMeas.segment(18 + i*3, 3) = this->sensors[i]->getMeas(typeData).segment(6,3);
				this->estMeas.segment(27 + i*4, 4) = this->sensors[i]->getMeas(typeData).tail(4);
			}
	}else if(typeData == REFeval)
	{
		for (int i =0; i < 3; i++)
		{
			this->refMeas.segment(i*3, 3) = this->sensors[i]->getMeas(typeData).head(3);
			this->refMeas.segment(9 + i*3, 3) = this->sensors[i]->getMeas(typeData).segment(3,3);
			this->refMeas.segment(18 + i*3, 3) = this->sensors[i]->getMeas(typeData).segment(6,3);
			this->refMeas.segment(27 + i*4, 4) = this->sensors[i]->getMeas(typeData).tail(4);
		}
	}
}
//void Limb::refreshLimbMeas(dataType const & typeData){
//	/*This function refreshes the measurement of the measurement vectors of the Limb object
//	 * getting them directly from each Sensor object
//	 * A call of refreshSensors() is needed before this method */
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
//	}else if (typeData == REFeval)
//	{
//		ScalarVector newMeas = ScalarMatrix::Zero(this->refMeas.rows(), 1);
//		for (unsigned int i = 0; i<this->sensors.size(); i++)
//		{
//			this->refMeas.segment(i*this->sensors[i].getNbMeasVar(), this->sensors[i].getNbMeasVar()) = this->sensors[i].getMeas(REF);
//		}
//	}else if (typeData == ESTeval)
//	{
//		ScalarVector newMeas = ScalarMatrix::Zero(this->refMeas.rows(), 1);
//		for (unsigned int i = 0; i<this->sensors.size(); i++)
//		{
//			this->estMeas.segment(i*this->sensors[i].getNbMeasVar(), this->sensors[i].getNbMeasVar()) = this->sensors[i].getMeas(EST);
//		}
//	}
//}

void Limb::setJointPos(ScalarVector const & newJointPos, dataType const & typeData){
	if (typeData == REF){
		this->refState.q = newJointPos;
		this->refTraj.qTraj.push_back(newJointPos);
	}else if (typeData == EST){
		this->estState.q = newJointPos;
		this->estTraj.qTraj.push_back(newJointPos);
	}else if (typeData == REFeval){
		this->refState.q = newJointPos;
	}else if (typeData == ESTeval){
		this->estState.q = newJointPos;
	}
}

void Limb::setJointVel(ScalarVector const & newJointPos, dataType const & typeData){
	if (typeData == REF){
		this->refState.dq = newJointPos;
		this->refTraj.dqTraj.push_back(newJointPos);
	}else if (typeData == EST){
		this->estState.dq = newJointPos;
		this->estTraj.dqTraj.push_back(newJointPos);
	}else if (typeData == REFeval){
		this->refState.dq = newJointPos;
	}else if (typeData == ESTeval){
		this->estState.dq = newJointPos;
	}
}

void Limb::setJointAcc(ScalarVector const & newJointPos, dataType const & typeData){
	if (typeData == REF){
		this->refState.ddq = newJointPos;
		this->refTraj.ddqTraj.push_back(newJointPos);
	}else if (typeData == EST){
		this->estState.ddq = newJointPos;
		this->estTraj.ddqTraj.push_back(newJointPos);
	}else if (typeData == REFeval){
		this->refState.ddq = newJointPos;
	}else if (typeData == ESTeval){
		this->estState.ddq = newJointPos;
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

//void Limb::setLimb_res_Jacobian(){
//
//	/* This function refreshes the residuals of each sensor, and refreshes the limbResJacobian
//	 * refreshSensors(REF) and refreshSensors(EST) have to be called before this function
//	 *
//	 * */
////	for (int i = 0; i<3; i++){
////		this->sensors[i].setResiduals_and_Jacobian(this->pinModel, this->pinData, this->refState, this->estState);
////		this->limbResJacobian.segment(i*84, 84) = this->sensors[i].getResidualsJacobian();
////	}
//	for (int i = 0; i<3; i++){
//		this->sensors[i].setResiduals_and_Jacobian(this->pinModel, this->pinData, this->refState, this->estState);
//		this->limbResJacobian.segment(i*84, 84) = this->sensors[i].getResidualsJacobian();
//	}
//}
ScalarVector Limb::getLimb_res_Jacobian() const{
		return this->limbResJacobian;
}
//
//void Limb::setResidualCostFunc(){
//	ADVector X = ADMatrix::Zero(this->nb_states*3,1);
//	ADVector refMeas = ADMatrix::Zero(this->nb_meas, 1);
//
//	Independent(X, refMeas);
//	ADVector sensorsResiduals = ADMatrix::Zero(this->nb_sensors,1); //Contains the residuals of each sensor
//	ADVector q = ADMatrix::Zero(this->nb_states,1);
//	ADVector dq = ADMatrix::Zero(this->nb_states,1);
//	ADVector ddq = ADMatrix::Zero(this->nb_states,1);
//	ADVector X_vel = ADMatrix::Zero(this->nb_states*2,1);
//	ADVector X_acc = ADMatrix::Zero(this->nb_states*3,1);
//
//	x_to_q_dq_ddq<ADVector>(X, q, dq, ddq);
//	q_dq_to_x<ADVector>(X_vel, q, dq);
//	q_dq_ddq_to_x<ADVector>(X_acc, q, dq, ddq);
//
//	for(int i =0; i<3; i++)
//	{
//
//		ADVector Pos = refMeas.segment(0 + i*13, 3);
//		ADVector Gyr = refMeas.segment(3 + i*13, 3);
//	    ADVector Acc = refMeas.segment(6 + i*13, 3);
//	    ADVector Quat = refMeas.segment(9 + i*13, 4);
//
//		this->sensors[i]->posCostFunAD.new_dynamic(Pos);
//		this->sensors[i]->gyrCostFunAD.new_dynamic(Gyr);
//		this->sensors[i]->accCostFunAD.new_dynamic(Acc);
//		this->sensors[i]->quatCostFunAD.new_dynamic(Quat);
//
//		sensorsResiduals(i) = (this->sensors[i]->posCostFunAD.Forward(0,q) +
//							 this->sensors[i]->gyrCostFunAD.Forward(0,X_vel) +
//							 this->sensors[i]->accCostFunAD.Forward(0,X_acc) +
//							 this->sensors[i]->quatCostFunAD.Forward(0,q))(0);
//	}
//	ADVector finalResidual(1);
//
//	finalResidual(0) = sensorsResiduals.sum();
//
//	ADFun<Scalar> residualFun(X, finalResidual);
//
//	this->residualCostFunc = std::move(residualFun);
//}

void Limb::setResidualCostFuncPos(){
	/** -------- ONLY POSITIONS ---------**/
	ADVector q = ADMatrix::Zero(this->nb_states*3,1);
	ADVector refMeas = ADMatrix::Zero(9, 1);

	Independent(q, refMeas);
	ADVector sensorsResiduals = ADMatrix::Zero(this->nb_sensors,1); //Contains the residuals of each sensor

	for(int i =0; i<3; i++)
	{

		ADVector Pos = refMeas.segment(0 + i*3, 3);

		this->sensors[i]->posCostFunAD.new_dynamic(Pos);
		sensorsResiduals(i) = (this->sensors[i]->posCostFunAD.Forward(0,q))(0);
	}
	ADVector finalResidual(1);
	finalResidual(0) = sensorsResiduals.sum();

	ADFun<Scalar> residualFun(q, finalResidual);
	this->residualCostFunc = std::move(residualFun);
}

void Limb::setResidualCostFuncPosQuat(){
	/** -------- ONLY POSITIONS ---------**/
	ADVector q = ADMatrix::Zero(this->nb_states*3,1);
	ADVector refMeas = ADMatrix::Zero(9 + 12, 1);

	Independent(q, refMeas);
	ADVector sensorsResiduals = ADMatrix::Zero(this->nb_sensors,1); //Contains the residuals of each sensor
	//ADVector q2 = q.head(13);
	for(int i =0; i<3; i++)
	{

		ADVector Pos = refMeas.segment(0 + i*3, 3);
		ADVector Quat = refMeas.segment(9 + i*4, 4);
		this->sensors[i]->posCostFunAD.new_dynamic(Pos);
		this->sensors[i]->quatCostFunAD.new_dynamic(Quat);
		sensorsResiduals(i) = (this->sensors[i]->posCostFunAD.Forward(0,q) + this->sensors[i]->quatCostFunAD.Forward(0,q))(0);
	}
	ADVector finalResidual(1);

	finalResidual(0) = sensorsResiduals.sum();

	ADFun<Scalar> residualFun(q, finalResidual);

	this->residualCostFunc = std::move(residualFun);
}

void Limb::setResidualCostFuncPosQuatQvel(){
	/** -------- ONLY POSITIONS ---------**/
	ADVector q = ADMatrix::Zero(this->nb_states*3,1);
	ADVector refMeas = ADMatrix::Zero(9 + 12 + 13, 1);

	Independent(q, refMeas);
	ADVector sensorsResiduals = ADMatrix::Zero(this->nb_sensors,1); //Contains the residuals of each sensor
	//ADVector q2 = q.head(13);
	ADVector q_prev = refMeas.tail(13);
	for(int i =0; i<3; i++)
	{

		ADVector Pos = refMeas.segment(0 + i*3, 3);
		ADVector Quat = refMeas.segment(9 + i*4, 4);
		this->sensors[i]->posCostFunAD.new_dynamic(Pos);
		this->sensors[i]->quatCostFunAD.new_dynamic(Quat);
		sensorsResiduals(i) = (this->sensors[i]->posCostFunAD.Forward(0,q) + this->sensors[i]->quatCostFunAD.Forward(0,q))(0);
	}
	ADVector cst = ADVector::Zero(13,1);
//	ADScalar newVal = 3;
//	for(int i = 0; i < c st.rows(), i++)
//	{
//		cst(i) = newVal;
//	}
	ADVector finalResidual(1);
	ADVector dq = (q.head(13) - q_prev)/dT;
	ADScalar regul_dq = (dq - cst).squaredNorm();
	finalResidual(0) = sensorsResiduals.sum() + 1e-10*regul_dq;

	ADFun<Scalar> residualFun(q, finalResidual);

	this->residualCostFunc = std::move(residualFun);
}

void Limb::setResidualCostFuncPosQuatGyr(){
	/** -------- ONLY POSITIONS ---------**/
	ADVector X = ADMatrix::Zero(this->nb_states*3,1);
	ADVector refMeas = ADMatrix::Zero(9 + 9 + 12 + 13, 1);

	Independent(X, refMeas);
	ADVector sensorsResiduals = ADMatrix::Zero(this->nb_sensors,1); //Contains the residuals of each sensor
	//ADVector q = X.head(13);
	for(int i =0; i<3; i++)
	{

		ADVector Pos = refMeas.segment(0 + i*3, 3);
		ADVector Gyr = refMeas.segment(9 + i*3, 3);
		ADVector Quat = refMeas.segment(18 + i*4, 4);
		this->sensors[i]->posCostFunAD.new_dynamic(Pos);
		this->sensors[i]->gyrCostFunAD.new_dynamic(Gyr);
		this->sensors[i]->quatCostFunAD.new_dynamic(Quat);
		sensorsResiduals(i) = (this->sensors[i]->posCostFunAD.Forward(0,X) + this->sensors[i]->quatCostFunAD.Forward(0,X) + this->sensors[i]->gyrCostFunAD.Forward(0,X))(0);
		//sensorsResiduals(i) = (this->sensors[i]->gyrCostFunAD.Forward(0,X))(0);

	}
	ADVector finalResidual(1);

	finalResidual(0) = sensorsResiduals.sum();

	ADFun<Scalar> residualFun(X, finalResidual);

	this->residualCostFunc = std::move(residualFun);
}

void Limb::setResidualCostFuncPosQuatAcc(){
	/** -------- ONLY POSITIONS ---------**/
	ADVector X = ADMatrix::Zero(this->nb_states*3,1);
	ADVector refMeas = ADMatrix::Zero(9 + 9 + 12, 1);

	Independent(X, refMeas);
	ADVector sensorsResiduals = ADMatrix::Zero(this->nb_sensors,1); //Contains the residuals of each sensor
	ADVector q = X.head(13);
	for(int i =0; i<3; i++)
	{

		ADVector Pos = refMeas.segment(0 + i*3, 3);
		ADVector Acc = refMeas.segment(9 + i*3, 3);
		ADVector Quat = refMeas.segment(18 + i*4, 4);
		this->sensors[i]->posCostFunAD.new_dynamic(Pos);
		this->sensors[i]->accCostFunAD.new_dynamic(Acc);
		this->sensors[i]->quatCostFunAD.new_dynamic(Quat);
		sensorsResiduals(i) = (this->sensors[i]->posCostFunAD.Forward(0,q) + this->sensors[i]->quatCostFunAD.Forward(0,q) + this->sensors[i]->accCostFunAD.Forward(0,X))(0);
	}
	ADVector finalResidual(1);

	finalResidual(0) = sensorsResiduals.sum();

	ADFun<Scalar> residualFun(X, finalResidual);

	this->residualCostFunc = std::move(residualFun);
}

//void Limb::setResidualCostFunc(){
//	ADVector X = ADMatrix::Zero(this->nb_states*3,1);
//	ADVector refMeas = ADMatrix::Zero(this->nb_meas, 1);
//
//	Independent(X, refMeas);
//	ADVector sensorsResiduals = ADMatrix::Zero(this->nb_sensors,1); //Contains the residuals of each sensor
//	ADVector q = ADMatrix::Zero(this->nb_states,1);
//	ADVector dq = ADMatrix::Zero(this->nb_states,1);
//	ADVector ddq = ADMatrix::Zero(this->nb_states,1);
//	ADVector X_vel = ADMatrix::Zero(this->nb_states*2,1);
//	ADVector X_acc = ADMatrix::Zero(this->nb_states*3,1);
//
//	x_to_q_dq_ddq<ADVector>(X, q, dq, ddq);
//	q_dq_to_x<ADVector>(X_vel, q, dq);
//	q_dq_ddq_to_x<ADVector>(X_acc, q, dq, ddq);
//
//	for(int i =0; i<this->nb_sensors; i++)
//	{
//
//		ADVector Pos = refMeas.segment(0 + i*(this->nb_meas/this->nb_sensors), 3);
//		ADVector Gyr = refMeas.segment(3 + i*(this->nb_meas/this->nb_sensors), 3);
//	    ADVector Acc = refMeas.segment(6 + i*(this->nb_meas/this->nb_sensors), 3);
//	    ADVector Quat = refMeas.segment(9 + i*(this->nb_meas/this->nb_sensors), 4);
//
//		this->sensors[i].posCostFunAD.new_dynamic(Pos);
//		this->sensors[i].gyrCostFunAD.new_dynamic(Gyr);
//		this->sensors[i].accCostFunAD.new_dynamic(Acc);
//		this->sensors[i].quatCostFunAD.new_dynamic(Quat);
//
//		sensorsResiduals(i) = (this->sensors[i].posCostFunAD.Forward(0,q) +
//							 this->sensors[i].gyrCostFunAD.Forward(0,X_vel) +
//							 this->sensors[i].accCostFunAD.Forward(0,X_acc) +
//							 this->sensors[i].quatCostFunAD.Forward(0,q))(0);
//	}
//	ADVector finalResidual(1);
//
//	finalResidual(0) = sensorsResiduals.sum();
//
//	ADFun<Scalar> residualFun(X, finalResidual);
//
//	this->residualCostFunc = std::move(residualFun);
//}

ScalarVector Limb::getResidual(ScalarVector const & X, ScalarVector const & refMeas){
	/*This function takes as input the state vector and the referenceMeasurement, refreshes the residual of the Limb
	 * and returns it */
	this->residualCostFunc.new_dynamic(refMeas);
	this->residual = this->residualCostFunc.Forward(0,X);
	return this->residual;
}

ScalarVector Limb::getResidualJacobian(ScalarVector const & X, ScalarVector const & refMeas){
	this->residualCostFunc.new_dynamic(refMeas);
	this->residualJacobian = this->residualCostFunc.Jacobian(X);
	return this->residualJacobian;
}

//void Limb::setJointNumDerivatives(dataType const & typeData){
//	if (this->estTraj.qTraj.size() >= 2 && typeData == EST){
//		this->estState.dq = (this->estTraj.qTraj[this->estTraj.qTraj.size()-1] -  this->estTraj.qTraj[this->estTraj.qTraj.size()-2])/dT;
//		this->estTraj.dqTraj.push_back(this->estState.dq);
//		this->estState.ddq = (this->estTraj.dqTraj[this->estTraj.dqTraj.size()-1] -  this->estTraj.dqTraj[this->estTraj.dqTraj.size()-2])/dT;
//		this->estTraj.ddqTraj.push_back(this->estState.ddq);
//	}else if (this->estTraj.qTraj.size() >= 2 && typeData == REF){
//		this->refState.dq = (this->refTraj.qTraj[this->refTraj.qTraj.size()-1] -  this->refTraj.qTraj[this->refTraj.qTraj.size()-2])/dT;
//		this->refTraj.dqTraj.push_back(this->refState.dq);
//		this->refState.ddq = (this->refTraj.dqTraj[this->refTraj.dqTraj.size()-1] -  this->refTraj.dqTraj[this->refTraj.dqTraj.size()-2])/dT;
//		this->refTraj.ddqTraj.push_back(this->refState.ddq);
//	}else if(this->estTraj.qTraj.size() < 2 && (typeData == REF || typeData == EST)){
//		this->estTraj.dqTraj.push_back(this->estState.dq);
//		this->estTraj.ddqTraj.push_back(this->estState.ddq);
//		this->refTraj.dqTraj.push_back(this->refState.dq);
//		this->refTraj.ddqTraj.push_back(this->refState.ddq);
//
//	}else if (this->estTraj.qTraj.size() >= 1 && typeData == REFeval){
////		this->refState.dq = (this->refTraj.qTraj[this->refTraj.qTraj.size()-1] -  this->refTraj.qTraj[this->refTraj.qTraj.size()-2])/Te;
////		this->refState.ddq = (this->refTraj.dqTraj[this->refTraj.dqTraj.size()-1] -  this->refTraj.dqTraj[this->refTraj.dqTraj.size()-2])/Te;
//
//		this->refState.dq = (this->refState.q -  this->refTraj.qTraj[this->refTraj.qTraj.size()-1])/dT;
//		//std::cout << "Ref dq : " << this->refState.q.transpose() << std::endl;
//		this->refState.ddq = (this->refState.dq -  this->refTraj.dqTraj[this->refTraj.dqTraj.size()-1])/dT;
//	}else if (this->estTraj.qTraj.size() >= 1 && typeData == ESTeval){
//		this->estState.dq = (this->estState.q -  this->estTraj.qTraj[this->estTraj.qTraj.size()-1])/dT;
//		//std::cout << "Est dq : " << this->estState.q.transpose() << std::endl;
//		this->estState.ddq = (this->estState.dq -  this->estTraj.dqTraj[this->estTraj.dqTraj.size()-1])/dT;
//	}
//}

void Limb::setJointNumDerivatives(ScalarVector const & q_es, dataType const & typeData){
//	std::cout << "q_es : " << q_es.transpose() << std::endl;
//	std::cout << "dT : " << dT << std::endl;
//	std::cout << "this->estTraj.qTraj[this->estTraj.qTraj.size()-1] : " << this->estTraj.qTraj[this->estTraj.qTraj.size()-1].transpose() << std::endl;
//	std::cout << "(q_es - q_es(t-1))/dT : " << ((q_es - this->estTraj.qTraj[this->estTraj.qTraj.size()-1])/dT).transpose() << std::endl;
//	ScalarVector q_es_prev = this->estTraj.qTraj[this->estTraj.qTraj.size()-1];
////	std::cout << "q_es_prev : " << q_es_prev.transpose() << std::endl;
//	ScalarVector zeez = q_es - q_es_prev;
//	std::cout << "zeez : " << zeez.transpose() << std::endl;
//	std::cout << "zeez/dT : " << zeez/dT << std::endl;
	if(typeData == ESTeval)
	{
		this->estState.dq = (q_es - this->estTraj.qTraj[this->estTraj.qTraj.size()-1])/dT;
		this->estState.ddq = (this->estState.dq - this->estTraj.dqTraj[this->estTraj.dqTraj.size()-1])/dT;
	}else if(typeData == EST)
	{
		this->estState.dq = (q_es - this->estTraj.qTraj[this->estTraj.qTraj.size()-2])/dT;
		this->estTraj.dqTraj.push_back(this->estState.dq);
//		std::cout << "Size : " << this->estTraj.dqTraj.size() << std::endl;
//		std::cout << this->estTraj.dqTraj[this->estTraj.dqTraj.size()-2].transpose() << std::endl;
		this->estState.ddq = (this->estState.dq - this->estTraj.dqTraj[this->estTraj.dqTraj.size()-2])/dT;
		this->estTraj.ddqTraj.push_back(this->estState.ddq);
	}
}

void Limb::setJointNumDerivatives(){
	if (this->estTraj.qTraj.size() >= 2){
		this->estState.dq = (this->estTraj.qTraj[this->estTraj.qTraj.size()-1] -  this->estTraj.qTraj[this->estTraj.qTraj.size()-2])/dT;
		this->estTraj.dqTraj.push_back(this->estState.dq);
		this->estState.ddq = (this->estTraj.dqTraj[this->estTraj.dqTraj.size()-1] -  this->estTraj.dqTraj[this->estTraj.dqTraj.size()-2])/dT;
		this->estTraj.ddqTraj.push_back(this->estState.ddq);
		this->refState.dq = (this->refTraj.qTraj[this->refTraj.qTraj.size()-1] -  this->refTraj.qTraj[this->refTraj.qTraj.size()-2])/dT;
		this->refTraj.dqTraj.push_back(this->refState.dq);
		this->refState.ddq = (this->refTraj.dqTraj[this->refTraj.dqTraj.size()-1] -  this->refTraj.dqTraj[this->refTraj.dqTraj.size()-2])/dT;
		this->refTraj.ddqTraj.push_back(this->refState.ddq);
	}else if (this->estTraj.qTraj.size() <= 1){
		this->estTraj.dqTraj.push_back(this->estState.dq);
		this->estTraj.ddqTraj.push_back(this->estState.ddq);
		this->refTraj.dqTraj.push_back(this->refState.dq);
		this->refTraj.ddqTraj.push_back(this->refState.ddq);
	}
}

ScalarVector Limb::getSensorsPos(dataType const & typeData) const{
	ScalarVector position = ScalarMatrix::Zero(this->nb_sensors*3, 1);
	if(typeData == REF || typeData == REFeval)
	{
		for (int i =0; i < 3; i++)
		{
			position.segment(i*3, 3) = this->sensors[i]->getMeas(typeData).head(3);
		}
	}else if(typeData == EST || typeData == ESTeval)
	{
		for (int i =0; i < 3; i++)
		{
			position.segment(i*3, 3) = this->sensors[i]->getMeas(typeData).head(3);
		}
	}
	return position;
}

ScalarVector Limb::getSensorsPosQuat(dataType const & typeData) const{

	ScalarVector posQuat = ScalarMatrix::Zero(this->nb_sensors*3 + this->nb_sensors*4, 1);
	if(typeData == REF || typeData == REFeval)
	{
		for (int i =0; i < 3; i++)
		{
			posQuat.segment(i*3, 3) = this->sensors[i]->getMeas(typeData).head(3);
			posQuat.segment(9 + i*4, 4) = this->sensors[i]->getMeas(typeData).tail(4);
		}
	}else if(typeData == EST || typeData == ESTeval)
	{
		for (int i =0; i < 3; i++)
		{
			posQuat.segment(i*3, 3) = this->sensors[i]->getMeas(typeData).head(3);
			posQuat.segment(9 + i*4, 4) = this->sensors[i]->getMeas(typeData).tail(4);
		}
	}
	return posQuat;
}

ScalarVector Limb::getSensorsPosQuatGyr(dataType const & typeData) const{

	ScalarVector posGyrQuat = ScalarMatrix::Zero(this->nb_sensors*3 + this->nb_sensors*3 + this->nb_sensors*4, 1);
	if(typeData == REF || typeData == REFeval)
	{
		for (int i =0; i < 3; i++)
		{
			posGyrQuat.segment(i*3, 3) = this->sensors[i]->getMeas(typeData).head(3);
			posGyrQuat.segment(9 + i*3, 3) = this->sensors[i]->getMeas(typeData).segment(3,3);
			posGyrQuat.segment(18 + i*4, 4) = this->sensors[i]->getMeas(typeData).tail(4);
		}
	}else if(typeData == EST || typeData == ESTeval)
	{
		for (int i =0; i < 3; i++)
		{
			posGyrQuat.segment(i*3, 3) = this->sensors[i]->getMeas(typeData).head(3);
			posGyrQuat.segment(9 + i*3, 3) = this->sensors[i]->getMeas(typeData).segment(3,3);
			posGyrQuat.segment(18 + i*4, 4) = this->sensors[i]->getMeas(typeData).tail(4);
		}
	}
	return posGyrQuat;
}

ScalarVector Limb::getSensorsPosQuatAcc(dataType const & typeData) const{

	ScalarVector posGyrQuat = ScalarMatrix::Zero(this->nb_sensors*3 + this->nb_sensors*3 + this->nb_sensors*4, 1);
	if(typeData == REF || typeData == REFeval)
	{
		for (int i =0; i < 3; i++)
		{
			posGyrQuat.segment(i*3, 3) = this->sensors[i]->getMeas(typeData).head(3);
			posGyrQuat.segment(9 + i*3, 3) = this->sensors[i]->getMeas(typeData).segment(6,3);
			posGyrQuat.segment(18 + i*4, 4) = this->sensors[i]->getMeas(typeData).tail(4);
		}
	}else if(typeData == EST || typeData == ESTeval)
	{
		for (int i =0; i < 3; i++)
		{
			posGyrQuat.segment(i*3, 3) = this->sensors[i]->getMeas(typeData).head(3);
			posGyrQuat.segment(9 + i*3, 3) = this->sensors[i]->getMeas(typeData).segment(6,3);
			posGyrQuat.segment(18 + i*4, 4) = this->sensors[i]->getMeas(typeData).tail(4);
		}
	}
	return posGyrQuat;
}
