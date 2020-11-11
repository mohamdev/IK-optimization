/*
 * kinematics.hpp
 *
 *  Created on: 21 Oct 2020
 *      Author: aladinedev2
 */

#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_
#include "utils.h"

/* ----------- ENUMS sensorType and dataType ------------- */
enum sensorType{
	POS, GYR, ACC, QUAT//, q, dq, ddq
};
enum dataType{
	EST, REF, MEAS, ESTeval, REFeval, MEASeval
};

/* -------------- STRUCT LimbData ----------------*/
/*
 * struct LimbData contains the data of the limb at the current time sample stored in ScalarVectors
 * */
typedef struct _JointStates{
	dataType typeDat;
	ScalarVector q;
	ScalarVector dq;
	ScalarVector ddq;
}JointStates;

/* ------------- STRUCT LimbTraj ----------------- */
/*
 * struct LimbTraj contains the whole trajectory data in a vector<ScalarVector>.
 * */
typedef struct _Trajectories{
	dataType typeDat;
	std::vector<ScalarVector> qTraj;
	std::vector<ScalarVector> dqTraj;
	std::vector<ScalarVector> ddqTraj;
	std::vector<ScalarVector> measTraj;
}Trajectories;

/* ------------ CLASS Sensor DECLARATION ------------- */
class Sensor {
private:
	ScalarVector meas; //contains the current value of the measurement
	ScalarVector refMeas;
	int nb_meas_var;
	int nb_states;
	int ID; //contains the id of the sensor, allowing to find it in pinocchio::Model
	Eigen::Vector3d g; //Gravity constant
	Eigen::Matrix3d R; //rotation matrix

public:
//	std::shared_ptr<ADFun<Scalar>>  posCostFun;
//	std::shared_ptr<ADFun<Scalar>> gyrCostFun;
//	std::shared_ptr<ADFun<Scalar>> accCostFun;
//	std::shared_ptr<ADFun<Scalar>> quatCostFun;
	ADFun<Scalar> posCostFun;
	ADFun<Scalar> gyrCostFun;
	ADFun<Scalar> accCostFun;
	ADFun<Scalar> quatCostFun;
	ADFun<AD<Scalar>, Scalar> posCostFunAD;
	ADFun<AD<Scalar>, Scalar> gyrCostFunAD;
	ADFun<AD<Scalar>, Scalar> accCostFunAD;
	ADFun<AD<Scalar>, Scalar> quatCostFunAD;
	ScalarVector jacPos;
	ScalarVector jacGyr;
	ScalarVector jacAcc;
	ScalarVector jacQuat;
	ScalarVector resJac;
	Scalar residual;
	std::vector<Scalar> residuals;
	Sensor();
	Sensor(Model const & model, int const & nb_sensor_var, int const & nb_state_variables, std::string ID);
	~Sensor();

	ScalarVector getMeas(dataType const & typeData) const;
	void setMeas(ScalarVector const & newValue);
	void setMeas(Model const & pinModel, Data & pinData, JointStates const & dataLimb);
	void setMeas(Model const & pinModel, Data & pinData, ScalarVector const & q);
	void setMeas(Model const & pinModel, Data & pinData, ScalarVector const & q, ScalarVector const & dq);
	void setRefMeas(ScalarVector const & newValue);
	void setPosADFun(ADModel const & pinADModel, ADData & pinADData);
	void setGyrADFun(ADModel const & pinADModel, ADData & pinADData);
	void setAccADFun(ADModel const & pinADModel, ADData & pinADData);
	void setQuatADFun(ADModel const & pinADModel, ADData & pinADData);
	void setADFuns(ADModel const & pinADModel, ADData & pinADData);

	ScalarMatrix getSensorJacobian(ADModel const & pinADModel, ADData & pinADData) const;
	void setResiduals_and_Jacobian(Model const & pinModel, Data & pinData, JointStates const & refStates, JointStates const & estStates);
	ScalarVector getResidualsJacobian() const;
	int getID() const;
	void setID(std::string newID);

	int getNbMeasVar();


	dataType getDataType() const;
	void setDataType(dataType typeData);

	int getSize() const;
};

/* ------------ CLASS Limb DECLARATION ------------- */
class Limb {
private:
	ScalarVector refMeas;
	ScalarVector estMeas;
public:
	ADFun<Scalar> residualCostFunc;
	std::vector<Scalar> t;
	std::vector<int> timesample;
	Trajectories estTraj;
	Trajectories refTraj;
	JointStates estState;
	JointStates refState;
	//std::vector<std::shared_ptr<Sensor>> sensors;
	std::vector<shared_ptr<Sensor>> sensors;
	Model pinModel;
	Data pinData;
	ADModel CppADModel;
	ADData CppADData;
	ScalarVector limbResJacobian;
	ScalarVector residualJacobian;
	ScalarVector residual;
	int nb_meas;
	int nb_states;
	int nb_sensors;
//
//	ADFun<Scalar> posCostFunc;
//	ADFun<Scalar> gyrCostFunc;
//	ADFun<Scalar> accCostFunc;
//	ADFun<Scalar> quatCostFunc;



	Limb();
	Limb(string const & urdf_filename, int const & nb_states, int const & nb_measurements, int const & nb_sensors);
	~Limb();

//	void setPosCostFunc();
//	void setGyrCostFunc();
//	void setAccCostFunc();
//	void setQuatCostFunc();

	void setDataDimensions(int const & nb_states, int const & nb_measurements);

	void refreshSensors(dataType const & typeData);
	ScalarVector getMeas(dataType const & typeData) const;
	void refreshLimbMeas(dataType const & typeData);

	void setJointPos(ScalarVector const & newJointPos, dataType const & typeData);
	ScalarVector getJointPos(dataType const & typeData) const;

	void setJointVel(dataType const & typeData);
	ScalarVector getJointVel(dataType const & typeData) const;

	void setJointAcc(dataType const & typeData);
	ScalarVector getJointAcc(dataType const & typeData) const;

	void setJointPosVelAcc(dataType const & typeData, ScalarVector const & q, ScalarVector const & dq, ScalarVector const & ddq);

	void addSensor(int const & nb_sensor_var, string ID);

	void setLimb_res_Jacobian();
	ScalarVector getLimb_res_Jacobian() const;

	void setResidualCostFunc();
	void setResidualCostFuncPos();
	void setResidualCostFuncPosQuat();
	ScalarVector getResidual(ScalarVector const & X, ScalarVector const & refMeas);
	ScalarVector getResidualJacobian(ScalarVector const & X, ScalarVector const & refMeas);

	void setJointNumDerivatives(dataType const & typeData);
	void setJointNumDerivatives();

	ScalarVector getSensorsPos(dataType const & typeData) const;
	ScalarVector getSensorsPosQuat(dataType const & typeData) const;
};

#endif /* KINEMATICS_HPP_ */
