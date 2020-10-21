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
typedef struct _LimbData{
	dataType typeDat;
	ScalarVector q;
	ScalarVector dq;
	ScalarVector ddq;
	ScalarVector meas;
}LimbData;

/* ------------- STRUCT LimbTraj ----------------- */
/*
 * struct LimbTraj contains the whole trajectory data in a vector<ScalarVector>.
 * */
typedef struct _LimbTraj{
	dataType typeDat;
	vector<ScalarVector> qTraj;
	vector<ScalarVector> dqTraj;
	vector<ScalarVector> ddqTraj;
	vector<ScalarVector> measTraj;
}LimbTraj;

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

/* ------------ CLASS Limb DECLARATION ------------- */
class Limb {
private:
	vector<Sensor> sensors;
	Model pinModel;
	Data pinData;
	LimbData esData;
	LimbData refData;
	LimbTraj esTraj;
	LimbTraj refTraj;
public:
	Limb(string const & urdf_filename, int const & nb_states, int const & nb_measurements);
	~Limb();
};



#endif /* KINEMATICS_HPP_ */
