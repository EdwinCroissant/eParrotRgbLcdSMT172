/* Copyright (C) 2017 Edwin Croissant
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the version 3 GNU General Public License as
 * published by the Free Software Foundation.
 */

#ifndef MYENUMS_H_
#define MYENUMS_H_

/*
 * To use enumerated types as parameters for functions the
 * declaration must be done in a separate header file.
 */


/*----( pin assignments )----*/

enum pins {
	pinBeeper = 2,	// Passive beeper use tone library
	pinLed = 13,
	pinBoilerPressure = A0,	// MPXV7002DP or equivalent
	pinBoilerAlarmDisable = A1,
	pinCondenserAlarmDisable = A2,
	pinCondenser = 6,		// DS18B20 Only
	pinBoiler = 7,			// DS18B20 Only
	pinVapor = 8,			// DS18B20 Only
	pinCS = 9,				// SD card cs
};

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

#endif /* MYENUMS_H_ */
