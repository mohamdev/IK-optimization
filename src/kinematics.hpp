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
	ScalarVector meas; //contains the current value of the measurement
	int nb_meas_var;
	int ID; //contains the id of the sensor, allowing to find it in pinocchio::Model
	dataType typeDat;
	Eigen::Vector3d g; //Gravity constant
	Eigen::Matrix3d R; //rotation matrix
public:
	Sensor();
	Sensor(Model const & model, dataType const & typeData, int const & nb_meas_variables, std::string ID);
	~Sensor();

	ScalarVector getMeas() const;
	void setMeas(ScalarVector const & newValue);
	void setMeas(Model const & pinModel, Data & pinData, JointStates const & dataLimb);

	int getID() const;
	void setID(std::string newID);

	dataType getDataType() const;
	void setDataType(dataType typeData);

	int getSize() const;
};

/* ------------ CLASS Limb DECLARATION ------------- */
class Limb {
private:
	vector<Sensor> estSensors;
	vector<Sensor> refSensors;
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

	void refreshSensors(dataType const & typeData);
	ScalarVector getMeas(dataType const & typeData) const;
	void refreshMeasVector(dataType const & typeData);
	//ADD refresh states

	void addSensor(int const & nb_sensor_variables, string ID);
};



#endif /* KINEMATICS_HPP_ */
