/*
 * kinematics.cpp
 *
 *  Created on: 21 Oct 2020
 *      Author: aladinedev2
 */

#include "kinematics.hpp"


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



/* -------------- CLASS Limb IMPLEMENTATION -------------*/

Limb::Limb(string const & urdf_filename, int const & nb_states, int const & nb_measurements){

    pinocchio::urdf::buildModel(urdf_filename, this->pinModel);
    //this->pinData = Data(this->pinModel);
    //initlaizeLimbTraj() (esTraj and refTraj)
    //initializeLimbData() (esData and refData);

}






