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
	POS, GYR, ACC, QUAT
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
	vector<ScalarVector> qTraj;
	vector<ScalarVector> dqTraj;
	vector<ScalarVector> ddqTraj;
	vector<ScalarVector> measTraj;
}Trajectories;

/* ------------ CLASS Sensor DECLARATION ------------- */
class Sensor {
private:
	ScalarVector value; //contains the current value of the measurement
	std::string ID; //contains the id of the sensor, allowing to find it in pinocchio::Model
	sensorType typeSens;
	dataType typeDat;
public:
	Sensor();
	Sensor(sensorType const & typeSensor, dataType const & typeData);
	~Sensor();

	ScalarVector getValue() const;
	void setValue(ScalarVector const & newValue);

	std::string getID() const;
	void setID(std::string newID);

	sensorType getSensType() const;
	void setSensType(sensorType typeSensor);

	dataType getDataType() const;
	void setDataType(dataType typeData);
};

/* ------------ CLASS Vimu DECLARATION -------------*/
class Vimu {
private:
	ScalarVector meas;
	int ID; //ID used to identify the sensor in pinocchio
	Eigen::Vector3d g; //Gravity constant
	Eigen::Matrix3d R; //rotation matrix
public:
	Vimu();
	Vimu(int const & nb_meas_variables, Model const & model, std::string ID);
	~Vimu();

	void setMeas(Model * pinModel, Data * pinData, JointStates const & dataLimb);
	ScalarVector getMeas() const;



};
/* ------------ CLASS Limb DECLARATION ------------- */
class Limb {
private:
	vector<Vimu> sensors;
	Model pinModel;
	Data pinData;
	JointStates estState;
	JointStates refState;
	Trajectories estTraj;
	Trajectories refTraj;
	ScalarVector refMeas;
	ScalarVector estMeas;
public:
	Limb(string const & urdf_filename, int const & nb_states, int const & nb_measurements);
	~Limb();

	void initializeLimbData(int const & nb_states, int const & nb_measurements);
	void refreshMeasurement();
};



#endif /* KINEMATICS_HPP_ */
