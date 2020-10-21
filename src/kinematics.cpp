/*
 * kinematics.cpp
 *
 *  Created on: 21 Oct 2020
 *      Author: aladinedev2
 */

#include "kinematics.hpp"


Sensor::Sensor(sensorType const & typeSensor, dataType const & typeData){
	this->setSensType(typeSensor);
	this->setDataType(typeData);
};


ScalarVector Sensor::getValue(){
	return this->value;
}
void Sensor::setValue(ScalarVector const & newValue){
	this->value = newValue;
}

std::string Sensor::getID(){
	return this->ID;
}
void Sensor::setID(std::string newID){
	this->ID = newID;
}

sensorType Sensor::getSensType(){
	return this->typeSens;
}
void Sensor::setSensType(sensorType typeSensor){
	this->typeSens = typeSensor;
}

dataType Sensor::getDataType(){
	return this->typeDat;
}
void Sensor::setDataType(dataType typeData){
	this->typeDat = typeData;
}
