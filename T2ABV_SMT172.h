/* Copyright (C) 2017 Edwin Croissant
 *
 * The array's of these functions are based on the Technical Paper 19 by
 * Herminio M. Brau dated April 1957.
 *
 * The tables are stored in a 24LC256 eeprom at I2C address 0x50
 *
 * Pressure influence is calculated with the Clausius Clapeyron relation.
 * For the liquid ABV the temperature is corrected with the change in boiling
 * point of water. At higher altitudes there will be some error for the high ABV values.
 * For the vapor ABV the temperature is corrected with the change in boiling
 * point of the azeotrope. At higher altitudes there will be some error
 * for the low ABV values.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the version 3 GNU General Public License as
 * published by the Free Software Foundation.
 */


#ifndef T2ABV_SMT172_H_
#define T2ABV_SMT172_H_

/*
 * -----Experimental-----
 * The vapor from a boiling water ethanol mixture is superheated.
 * In a CM still the vapor after passing the reflux condenser is
 * no longer superheated, resulting in a lower temperature.
 * #define HOTVAPOR 0 when the temperature sensor is placed after the
 * reflux condenser. Please note that at full reflux the vapor
 * temperature is lost.
 */

#define HOTVAPOR 1

float h2oBoilingPoint(float p) {
	// calculate the the boiling temperature of water for the measured pressure in Celsius
	float H2OInKelvin = -20330000 * 373.15
			/ (4157 * log(p / 1013.25) * 373.15 - 20330000);
	return H2OInKelvin - 273.15;
}

float azeotrope(float p) {
	// Calculate the the azeotrope for the measured pressure in Celsius
	float AzeotropeInKelvin = -19280000 * 351.324
			/ (4157 * log(p / 1013.25) * 351.324 - 19280000);
	return AzeotropeInKelvin - 273.15;
}

uint16_t eeRead16(uint16_t address) {
	union {
		uint8_t as8[2];
		uint16_t as16;
	} value;
	I2c._start();
	I2c._sendAddress(SLA_W(0x50));
	I2c._sendByte(highByte(address));
	I2c._sendByte(lowByte(address));
	I2c._start();
	I2c._sendAddress(SLA_R(0x50));
	I2c._receiveByte(1, &value.as8[0]);
	I2c._receiveByte(0, &value.as8[1]);
	I2c._stop();
	return value.as16;
}

#if HOTVAPOR == 1

float TtoLiquidABV(float T, float P) {
	// Calculate the index for the table (7818 is the the azeotrope at 1013.25 hPa and the
	// starting point of the table) in °cC
	int16_t IndexABV = int16_t((T + 100 - h2oBoilingPoint(P)) * 100 + 0.5) - 7818;
	if (IndexABV < 0) return float(IndexABV) * 0.01; // Below azeotrope
	if (IndexABV + 7818 > 10000) return 0; // Above 100 °C
	return float(eeRead16( 0 + IndexABV * 2)) / 10;
};

float TtoVaporABV(float T, float P) {
	// Calculate the index for the table (7818 is the the azeotrope at 1013.25 hPa and the
	// starting point of the table) in °cC
	int16_t IndexABV = int16_t((T + 78.174 - azeotrope(P))  * 100 + 0.5) - 7818;
	if (IndexABV < 0) return float(IndexABV) * 0.01; // Below azeotrope
	if (IndexABV + 7818 > 10000) return 0; // Above 100 °C
	return float(eeRead16( 0x110E + IndexABV * 2)) / 10;
};

#else

float TtoLiquidABV(float T, float P) {
	// Calculate the index for the table (7818 is the the azeotrope at 1013.25 hPa and the
	// starting point of the table) in °cC
	int16_t IndexABV = int16_t((T + 100 - h2oBoilingPoint(P)) * 100 + 0.5) - 7818;
	if (IndexABV < 0) return float(IndexABV) * 0.01; // Below azeotrope
	if (IndexABV + 7818 > 10000) return 0; // Above 100 °C
	return float(eeRead16( 0 + IndexABV * 2)) / 10;
};

float TtoVaporABV(float T, float P) {
	// Calculate the index for the table (7818 is the the azeotrope at 1013.25 hPa and the
	// starting point of the table) in °cC
	int16_t IndexABV = int16_t((T + 78.174 - azeotrope(P)) * 100 + 0.5) - 7818;
	if (IndexABV < 0) return float(IndexABV) * 0.01; // Below azeotrope
	if (IndexABV + 7818 > 10000) return 0; // Above 100 °C
	return float(eeRead16( 0 + IndexABV * 2)) / 10;
};

#endif /* HOTVAPOR */

#endif /* T2ABV_SMT172_H_ */
