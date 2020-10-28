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
	EST, REF, MEAS
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
	int nb_meas_var;
	int ID; //contains the id of the sensor, allowing to find it in pinocchio::Model
	dataType typeDat;
	Eigen::Vector3d g; //Gravity constant
	Eigen::Matrix3d R; //rotation matrix

public:
	ADFun<Scalar> posCostFun;
	ADFun<Scalar> gyrCostFun;
	ADFun<Scalar> accCostFun;
	ADFun<Scalar> quatCostFun;
	Sensor();
	Sensor(Model const & model, int const & nb_meas_variables, dataType const & typeData, std::string ID);
	~Sensor();

	ScalarVector getMeas() const;
	void setMeas(ScalarVector const & newValue);
	void setMeas(Model const & pinModel, Data & pinData, JointStates const & dataLimb);
	void setMeas(Model const & pinModel, Data & pinData, ScalarVector const & q);
	void setMeas(Model const & pinModel, Data & pinData, ScalarVector const & q, ScalarVector const & dq);
	void setPosADFun(ADModel const & pinADModel, ADData & pinADData);
	void setGyrADFun(ADModel const & pinADModel, ADData & pinADData);
	void setAccADFun(ADModel const & pinADModel, ADData & pinADData);
	void setQuatADFun(ADModel const & pinADModel, ADData & pinADData);
	void setADFuns(ADModel const & pinADModel, ADData & pinADData);
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
	JointStates estState;
	JointStates refState;
	ScalarVector refMeas;
	ScalarVector estMeas;
public:
	std::vector<Sensor> estSensors;
	std::vector<Sensor> refSensors;
	Model pinModel;
	Data pinData;
	ADModel CppADModel;
	ADData CppADData;
//
//	ADFun<Scalar> posCostFunc;
//	ADFun<Scalar> gyrCostFunc;
//	ADFun<Scalar> accCostFunc;
//	ADFun<Scalar> quatCostFunc;

	std::vector<Scalar> t;
	std::vector<int> timesample;
	Trajectories estTraj;
	Trajectories refTraj;

	Limb(string const & urdf_filename, int const & nb_states, int const & nb_measurements);
	~Limb();

	void setPosCostFunc();
	void setGyrCostFunc();
	void setAccCostFunc();
	void setQuatCostFunc();

	void setDataDimensions(int const & nb_states, int const & nb_measurements);

	void refreshAllSensors(dataType const & typeData);
	ScalarVector getMeas(dataType const & typeData) const;
	void refreshMeasVector(dataType const & typeData);

	void setJointPos(ScalarVector const & newJointPos, dataType const & typeData);
	ScalarVector getJointPos(dataType const & typeData) const;

	void setJointVel(dataType const & typeData);
	ScalarVector getJointVel(dataType const & typeData) const;

	void setJointAcc(dataType const & typeData);
	ScalarVector getJointAcc(dataType const & typeData) const;

	void setJointPosVelAcc(dataType const & typeData, ScalarVector const & q, ScalarVector const & dq, ScalarVector const & ddq);

	void addSensor(int const & nb_sensor_var, string ID);
};



#endif /* KINEMATICS_HPP_ */
