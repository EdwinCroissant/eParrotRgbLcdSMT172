/* Copyright (C) 2017 Edwin Croissant
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the version 3 GNU General Public License as
 * published by the Free Software Foundation.
 */

#ifndef MYPINSANDSETTINGS_H_
#define MYPINSANDSETTINGS_H_

/* -----------------IMPORTANT-----------------
 * In SdFatConfig.h change
 * #define USE_LONG_FILE_NAMES 1
 * into
 * #define USE_LONG_FILE_NAMES 0
 * to make the compiled sketch fit into the UNO R3
 */

/* -----------------IMPORTANT-----------------
 * Remove pin 9 from the LCD RGB 16x2 Keypad Shield when using the
 * Robotdyn Data logger shield as this pin is connected to the LCD
 * contrast trimmer.
 */

/*----( pin assignments )----*/

enum pins {
	pinBeeper = 2,	// Passive beeper use tone library
	pinLed = 13,
	pinBoilerAlarmEnable = A0,	// high is enabled, low is disabled
	pinVent2AlarmEnable = A1,	// high is enabled, low is disabled
	pinVent1AlarmEnable = A2,	// high is enabled, low is disabled
	pinBoilerPressure = A3,		// MPXV7002DP or equivalent
	pinVent2 = 5,				// DS18B20 only
	pinVent1 = 6,				// DS18B20 only
	pinBoiler = 7,				// DS18B20 only
	pinVapor = 8,				// SMT172 or DS18B20
	pinCS = 9,					// SD card cs Robotdyn Data Logger shield
//	pinCS = 10					// SD card cs Deek-Robot Data logging shield V1.0
};

/* ---( compiled settings )--- */

const int8_t ForesAlarm = -10; // 10 C below azeotrope
const int8_t VentAlarmTemperature = 60;
const float VentAlarmDeltaTemperature = 0.5;
const uint8_t BoilerAlarmPressure = 100;

#endif /* MYPINSANDSETTINGS_H_ */
