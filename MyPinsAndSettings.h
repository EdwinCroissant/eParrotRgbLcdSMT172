/* Copyright (C) 2017 Edwin Croissant
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the version 3 GNU General Public License as
 * published by the Free Software Foundation.
 */

#ifndef MYPINSANDSETTINGS_H_
#define MYPINSANDSETTINGS_H_

/*----( pin assignments )----*/

enum pins {
	pinBeeper = 2,	// Passive beeper use tone library
	pinLed = 13,
	pinBoilerAlarmEnable = A0,	// high is enabled, low is disabled
	pinVent2AlarmEnable = A1,	// high is enabled, low is disabled
	pinVent1AlarmEnable = A2,	// high is enabled, low is disabled
	pinBoilerPressure = A3,		// MPXV7002DP or equivalent
	pinVent2 = 5,				// DS18B20 Only
	pinVent1 = 6,				// DS18B20 Only
	pinBoiler = 7,				// DS18B20 Only
	pinVapor = 8,				// SMT172 or DS18B20
	pinCS = 9,					// SD card cs
};

/* ---( compiled settings )--- */

const int8_t ForesAlarm = -10; // 10 C below azeotrope
const int8_t VentAlarmTemperature = 60;
const float VentAlarmDeltaTemperature = 0.5;
const uint8_t BoilerAlarmPressure = 100;

#endif /* MYPINSANDSETTINGS_H_ */
