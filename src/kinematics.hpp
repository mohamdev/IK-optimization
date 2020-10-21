/*
 * kinematics.hpp
 *
 *  Created on: 21 Oct 2020
 *      Author: aladinedev2
 */

#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_
#include "utils.h"

enum sensorType{
	POS, GYR, ACC, QUAT
};
enum dataType{
	EST, REF, MEAS
};

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

	ScalarVector getValue();
	void setValue(ScalarVector const & newValue);

	std::string getID();
	void setID(std::string newID);

	sensorType getSensType();
	void setSensType(sensorType typeSensor);

	dataType getDataType();
	void setDataType(dataType typeData);
};



#endif /* KINEMATICS_HPP_ */
