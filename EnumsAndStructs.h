/* Copyright (C) 2017 Edwin Croissant
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the version 3 GNU General Public License as
 * published by the Free Software Foundation.
 */

#ifndef ENUMSANDSTRUCTS_H_
#define ENUMSANDSTRUCTS_H_
#include <SingleDS18B20.h>		//https://github.com/EdwinCroissantArduinoLibraries/SingleDS18B20

/*
 * To use enumerated types and structures as parameters for functions
 * their declarations must be done in a separate header file.
 */

/*----( Recognizable names for the sensor types )----*/
enum sensorType {
	NoSensor = 0,
	smt172 = 1,
	DS18B20 = 2,
};

/*----( Recognizable names for Sd status )----*/
enum createStatus {
	noCard,
	fullCard,
	fileOk
};

/*----( Recognizable names for the alarm status )----*/
enum alarmStatus {
	disabled,
	armed,
	triggered,
	acknowledged
};

enum healthAlarm {
	healthOk,
	healthBoiler,
	healthVent1,
	healthVent2
};

struct temperatureSensor {
	sensorType Type;
	float Temperature;
};

#endif /* ENUMSANDSTRUCTS_H_ */
