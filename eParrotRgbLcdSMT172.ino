/* Copyright (C) 2017 Edwin Croissant
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the version 3 GNU General Public License as
 * published by the Free Software Foundation.
 */
#include <I2C.h>				//https://github.com/rambo/I2C
#include <OneWire.h>			//https://github.com/bigjosh/OneWireNoResistor
#include <RgbLcdKeyShieldI2C.h>	//https://github.com/EdwinCroissantArduinoLibraries/RgbLcdKeyShieldI2C
#include <SdFat.h>				//https://github.com/greiman/SdFat
#include <SimpleBMP280I2C.h>	//https://github.com/EdwinCroissantArduinoLibraries/SimpleBMP280I2C
#include <SimpleDS1307I2C.h>	//https://github.com/EdwinCroissantArduinoLibraries/SimpleDS1307I2C
#include <SingleDS18B20.h>		//https://github.com/EdwinCroissantArduinoLibraries/SingleDS18B20
#include <SMT172.h>				//https://github.com/EdwinCroissantArduinoLibraries/SMT172
#include "myEnums.h"
#include "T2ABV_SMT172.h"

/* -----------------IMPORTANT-----------------
 * In SdFatConfig.h change
 * #define USE_LONG_FILE_NAMES 1
 * into
 * #define USE_LONG_FILE_NAMES 0
 * to make the compiled sketch fit into the UNO R3
 */


/*----( fixed alarm values )----*/
const int8_t ForesAlarm = -10;
const int8_t CondenserAlarmTemperature = 60;
const float CondenserAlarmDeltaTemperature = 0.5;
const uint8_t BoilerAlarmPressure = 100;

/*----( strings in flash )----*/
const char msgSplash1[] PROGMEM = "eParrot  RGB LCD";
const char msgSplash2[] PROGMEM = "V 0.07 S  (c) EC";
const char logFilename[] PROGMEM =  "RUN_00.CSV";
const char msgNoBaro[] PROGMEM = "No Barometer";
const char msgCanceled[] PROGMEM = "Canceled";
const char msgSaved[] PROGMEM = "Saved";
const char msgToLog[] PROGMEM = "Press S to log ";
const char msgToStop[] PROGMEM = "Press S to stop";
const char msgNoCard[] PROGMEM = "No card   ";
const char msgFullCard[] PROGMEM = "Card full ";
const char msghPa[] PROGMEM = " hPa";
const char msgdPa[] PROGMEM = " dPa";
const char msgBPWater[] PROGMEM = "BP Water";
const char msgNoValue[] PROGMEM = "--.-%";
const char msgNoSensor[] PROGMEM = "  Sensor error  ";
const char msgBoilerOffset[] PROGMEM = "Blr Offs";
const char msgVaporOffset[] PROGMEM = "Vpr Offs";
const char msgSilent[] PROGMEM = "Silent";
const char msgAudial[] PROGMEM = "Audial";
const char msgForesAlarm[] PROGMEM = " FS";
const char msgFores[] PROGMEM = "FORES";
const char msgCondenser[] PROGMEM = " Condenser vent";
const char msgBoilerPressure[] PROGMEM = "Boiler pressure";

/*----( make instances for the Dallas sensors, BMP280, etc. )----*/
OneWire pinBoilerSensor(pinBoiler), pinVaporSensor(pinVapor),
		pinCondenserSensor(pinCondenser);
SingleDS18B20 BoilerDS18B20(&pinBoilerSensor), VaporDS18B20(&pinVaporSensor),
		CondenserDS18B20(&pinCondenserSensor);
SimpleBMP280I2C baro(0x77); // for I2C address 0x77 Robotdyn
// SimpleBMP280I2C baro(0x76); // for I2C address 0x76 Thinary
RgbLcdKeyShieldI2C lcd;
SdFat sd;

/*-----( Declare Variables )-----*/
struct sensors {
	float BaroPressure;				// in hPa
//	float BaroTemperature;			// in C
	int16_t BoilerPressureRaw;
	int16_t BoilerPressure;			// in mmH2O (dPa)
	sensorType VaporType;
	float VaporTemperature;			// in C
	float VaporABV;					// in %
	float VaporAlarmSetpoint;		// in C
	float H2OBoilingPoint;			// in C
	sensorType BoilerType;
	float BoilerTemperature;		// in C
	float BoilerABV;				// in %
	float BoilerLastTemperature;	// in C
	int16_t WarmupTime;				// in minutes
	float CondenserLastTemperature;// in C
	float CondenserTemperature;	// in C
	sensorType CondenserType;
} Sensors;

union {
	uint8_t settingsArray[];
	struct {
		int16_t VaporOffset;		// in cC
		int16_t BoilerOffset;		// in cC
		uint8_t Alarm[4];			// in %
		uint8_t WarmedUp;			// in C
		int16_t BoilerPressureOffset;
	};
} Settings;

alarmStatus AlarmStatusVapor;
alarmStatus AlarmStatusCondenser;
alarmStatus AlarmStatusBoiler;

struct log {
	char name[11];
	bool isLogging;
} LogFile;

char lineBuffer[20];

void (*AutoPageFastRefresh)();
void (*AutoPageRefresh)();
void (*ReturnPage)();

bool Backlight = true;
bool FlashBacklight;
bool Silent;
uint32_t LastSMT172Update;
uint32_t LastSensorUpdate;
uint32_t AlternateBacklightUpdate;
uint32_t StartTimeLog;
uint32_t LastAlarmUpdate;
uint32_t LastWarmingupUpdate;
uint8_t CurrentAlarm = 4;

int16_t* offset;
int16_t oldOffset;
void (*PrintOffset)();

void saveSettings() {
	rtc.writeRam(0,Settings.settingsArray,sizeof(Settings));
}

void loadSettings() {
	rtc.readRam(0, sizeof(Settings), Settings.settingsArray);
}

// call back for file timestamps
void dateTime(uint16_t* date, uint16_t* time) {
	// get the time
	rtc.readClock();
	// return date using FAT_DATE macro to format fields
	*date = FAT_DATE(2000 + rtc.year, rtc.month, rtc.day);
	// return time using FAT_TIME macro to format fields
	*time = FAT_TIME(rtc.hour, rtc.minute, rtc.second);
}

// Helper function to initialize the sensors and start the conversion
void initVaporSensor() {
	SMT172::startTemperature(0.001);
	delay(5);
	if (SMT172::getStatus() != 2) {
		Sensors.VaporType = smt172;
	} else if (VaporDS18B20.read()) {
		VaporDS18B20.setResolution(SingleDS18B20::res12bit);
		VaporDS18B20.convert();
		Sensors.VaporType = DS18B20;
	} else {
		// disable internal pullup resistor
		pinMode(pinVapor, INPUT);
		digitalWrite(pinVapor, LOW);
		Sensors.VaporType = NoSensor;
	}
}

void initBoilerSensor() {
	if (BoilerDS18B20.read()) {
		BoilerDS18B20.setResolution(SingleDS18B20::res12bit);
		BoilerDS18B20.convert();
		Sensors.BoilerType = DS18B20;
	} else
		Sensors.BoilerType = NoSensor;
}

void initCondenserSensor() {
	if (CondenserDS18B20.read()) {
		CondenserDS18B20.setResolution(SingleDS18B20::res12bit);
		CondenserDS18B20.convert();
		Sensors.CondenserType = DS18B20;
	} else
		Sensors.CondenserType = NoSensor;
}

//The setup function is called once at startup of the sketch
void setup()
{
	// enable pullups for A1, A2
	pinMode(pinBoilerAlarmDisable, INPUT_PULLUP);
	pinMode(pinCondenserAlarmDisable, INPUT_PULLUP);

	I2c.begin();
	I2c.setSpeed(1);
	I2c.timeOut(10);
	lcd.begin();
	lcd.setColor(RgbLcdKeyShieldI2C::clWhite);

/* TODO test the alternative address before giving up */
	if (!baro.begin()) {
		lcd.printP(msgNoBaro);
		while (true) {
		};
	}

	// Initialize the temperature sensors
	initBoilerSensor();
	initVaporSensor();
	initCondenserSensor();

	// show splash
	lcd.printP(msgSplash1);
	lcd.setCursor(0,1);
	lcd.printP(msgSplash2);

	delay(3000);

	readSensors();
	LastSensorUpdate=millis();

	// check if time is set
	if (!rtc.readClock()) {
		rtc.clearRam();
		rtc.year = 17;
		rtc.month = 1;
		rtc.day = 1;
		rtc.hour = 12;
		rtc.minute = 0;
		rtc.second = 0;
		ReturnPage = showTimeInit;
		setTimeInit();
	} else
		showMainInit();

	loadSettings();
}

void doFunctionAtInterval(void (*callBackFunction)(), uint32_t *lastEvent,
		uint32_t Interval) {
	uint32_t now = millis();
	if ((now - *lastEvent) >= Interval) {
		*lastEvent = now;
		callBackFunction();
	}
}

// The loop function is called in an endless loop
void loop()
{
	doFunctionAtInterval(readSMT172, &LastSMT172Update, 250);
	doFunctionAtInterval(alternateBacklight, &AlternateBacklightUpdate, 500);
	doFunctionAtInterval(readSensors, &LastSensorUpdate, 1000);	// read the baro and DS18B20's every second
	doFunctionAtInterval(handleWarmingup, &LastWarmingupUpdate, 60000);	// check warming up every minute
	lcd.readKeys();
}

/* helper function to flash the backlight when Flashbacklight is true
 * TODO move to lcd library
 */
void alternateBacklight() {
	Backlight = !Backlight;
	if (FlashBacklight) {
		if (Backlight)
			lcd.setColor(RgbLcdKeyShieldI2C::clRed);
		else
			lcd.setColor(RgbLcdKeyShieldI2C::clViolet);
	}
}

void handleWarmingup() {
	float DeltaT;
	// calculate the warmup time
	if ((Sensors.BoilerType != NoSensor) && (Sensors.BoilerLastTemperature > 0)
			&& (Sensors.BoilerTemperature < Settings.WarmedUp)) {
		DeltaT = Sensors.BoilerTemperature - Sensors.BoilerLastTemperature;
		Sensors.WarmupTime = int16_t(constrain((float(Settings.WarmedUp)
					- Sensors.BoilerTemperature) / DeltaT + 0.5, 0, 5999));
	} else
		Sensors.WarmupTime = 0;

	Sensors.BoilerLastTemperature = Sensors.BoilerTemperature;
}

void handleAlarms() {
	bool BoilerAlarmCondition;
	bool CondenserAlarmCondition;
	bool VaporAlarmCondition;

	// check boiler first
	BoilerAlarmCondition = digitalRead(pinBoilerAlarmDisable) && Sensors.BoilerPressure > BoilerAlarmPressure;

	switch (AlarmStatusBoiler) {
	case disabled:
			AlarmStatusBoiler = armed;
		break;
	case armed:
		if (BoilerAlarmCondition) {
			AlarmStatusBoiler = triggered;
			showBoilerPressureAlarmInit();
		}
		break;
	case triggered:
		tone(pinBeeper, 440, 500);
		FlashBacklight = true;
		break;
	case acknowledged:
		FlashBacklight = false;
		if (BoilerAlarmCondition)
			lcd.setColor(RgbLcdKeyShieldI2C::clRed);
		else {
			AlarmStatusBoiler = armed;
			lcd.setColor(RgbLcdKeyShieldI2C::clWhite);
			showMainInit();
			}
		break;
	}

	if (BoilerAlarmCondition)
		return;

	// check condenser second
	CondenserAlarmCondition = digitalRead(pinCondenserAlarmDisable)
			&& (Sensors.CondenserType == NoSensor
					|| Sensors.CondenserTemperature > CondenserAlarmTemperature
					|| Sensors.CondenserTemperature
							- Sensors.CondenserLastTemperature
							> CondenserAlarmDeltaTemperature);

	switch (AlarmStatusCondenser) {
	case disabled:
			AlarmStatusCondenser = armed;
		break;
	case armed:
		if (CondenserAlarmCondition) {
			AlarmStatusCondenser = triggered;
			showCondenserFailureInit();
		}
		break;
	case triggered:
		tone(pinBeeper, 440, 500);
		FlashBacklight = true;
		break;
	case acknowledged:
		FlashBacklight = false;
		if (CondenserAlarmCondition)
			lcd.setColor(RgbLcdKeyShieldI2C::clRed);
		else {
			AlarmStatusCondenser = armed;
			lcd.setColor(RgbLcdKeyShieldI2C::clWhite);
			showMainInit();
			}
		break;
	}

	if (CondenserAlarmCondition)
		return;

	VaporAlarmCondition = ((Sensors.VaporABV >= 0 && CurrentAlarm < 4
			&& Sensors.VaporABV < Settings.Alarm[CurrentAlarm])
			|| (Sensors.VaporABV < 0 && CurrentAlarm > 3
					&& Sensors.VaporABV > ForesAlarm));

	switch (AlarmStatusVapor) {
	case disabled:
		lcd.setColor(RgbLcdKeyShieldI2C::clWhite);
		break;
	case armed:
		if (VaporAlarmCondition)
			AlarmStatusVapor = triggered;
		if (Silent)
			lcd.setColor(RgbLcdKeyShieldI2C::clBlue);
		else
			lcd.setColor(RgbLcdKeyShieldI2C::clGreen);
		break;
	case triggered:
		FlashBacklight = true;
		if (!Silent)
			tone(pinBeeper, 440, 500);
		break;
	case acknowledged:
		FlashBacklight = false;
		if (!VaporAlarmCondition)
			AlarmStatusVapor = armed;
		lcd.setColor(RgbLcdKeyShieldI2C::clRed);
		break;
	}
}

int16_t calibratedAnalogInMilliVolt(int16_t value) {
	/* Measure internal reference */
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	delay(2); // Wait for Vref (+/- 1.1V) to settle
	ADCSRA |= _BV(ADSC); // Start conversion
	while (bit_is_set(ADCSRA,ADSC)); // measuring
	uint8_t low  = ADCL; // read ADCL first, this locks ADCH
	uint8_t high = ADCH; // unlocks both
	int32_t Vref = (high<<8) | low;
	return (1100L * value) / Vref;
}

void readSMT172() {
	if (Sensors.VaporType == smt172) {
		switch (SMT172::getStatus()) {
		case 0:
			break;
		case 1:
			Sensors.VaporTemperature = SMT172::getTemperature()
					+ float(Settings.VaporOffset) / 100;
			Sensors.VaporABV = TtoVaporABV(Sensors.VaporTemperature,
							Sensors.BaroPressure);
			SMT172::startTemperature(0.001);
			break;
		case 2:
			Sensors.VaporType = NoSensor;
		}
	}
	if (AutoPageFastRefresh)
		AutoPageFastRefresh();
}


void readSensors() {
	// Retrieve the current pressure in Pascal.
	Sensors.BaroPressure = float(baro.getPressure()) / 100;
	// calculate the boiling point of water
	Sensors.H2OBoilingPoint = h2oBoilingPoint(Sensors.BaroPressure);
	/*
	 * retrieve the boiler pressure
	 * TODO make calibration possible, the sensor output is 1V for each kPa,
	 * but the internal reference can deviate up to 10%
	 */
	Sensors.BoilerPressureRaw = calibratedAnalogInMilliVolt(analogRead(pinBoilerPressure));
	Sensors.BoilerPressure = (Sensors.BoilerPressureRaw - Settings.BoilerPressureOffset) / 10 ;

	if (Sensors.VaporType == DS18B20) {
		if (VaporDS18B20.read() && VaporDS18B20.convert()) {
			Sensors.VaporTemperature = VaporDS18B20.getTempAsC()
					+ float(Settings.VaporOffset) / 100;
			Sensors.VaporABV = TtoVaporABV(Sensors.VaporTemperature,
							Sensors.BaroPressure);
		} else
			Sensors.VaporType = NoSensor;
	}

	if (Sensors.BoilerType == DS18B20) {
		if (BoilerDS18B20.read() && BoilerDS18B20.convert()) {
			Sensors.BoilerTemperature = BoilerDS18B20.getTempAsC()
					+ float(Settings.BoilerOffset) / 100;
			Sensors.BoilerABV = TtoLiquidABV(Sensors.BoilerTemperature,
							Sensors.BaroPressure);
		} else
			Sensors.BoilerType = NoSensor;
	}

	if (Sensors.CondenserType == DS18B20) {
		if (CondenserDS18B20.read() && CondenserDS18B20.convert()) {
			Sensors.CondenserLastTemperature = Sensors.CondenserTemperature;
			Sensors.CondenserTemperature = CondenserDS18B20.getTempAsC();
		} else
			Sensors.CondenserType = NoSensor;
	}

	if (Sensors.VaporType == NoSensor)
		initVaporSensor();

	if (Sensors.BoilerType == NoSensor)
		initBoilerSensor();

	if (Sensors.CondenserType == NoSensor)
		initCondenserSensor();

	if (AutoPageRefresh)
		AutoPageRefresh();

	if (LogFile.isLogging)
		LogFile.isLogging = writeDataToFile();

	handleAlarms();
}

createStatus createFile() {
	strcpy_P(LogFile.name, logFilename);
	if (!sd.begin(pinCS, SPI_QUARTER_SPEED))
		return noCard;

	// create a new file
	for (uint8_t i = 0; i < 100; ++i) {
		LogFile.name[4] = i / 10 + '0';
		LogFile.name[5] = i % 10 + '0';
		// only open a new file if it doesn't exist
		if (!sd.exists(LogFile.name)) {
			// set the file time stamp callback
			SdFile::dateTimeCallback(dateTime);
			File dataFile = sd.open(LogFile.name, FILE_WRITE);
			dataFile.print(LogFile.name);
			dataFile.print(' ');
			sprintf(lineBuffer, "20%.2hd-%.2hd-%.2hdT%.2hd:%.2hd:%.2hd",
					rtc.year, rtc.month, rtc.day, rtc.hour, rtc.minute,
					rtc.second);
			dataFile.println(lineBuffer);
			dataFile.close();
			// timestamp is creation time
			SdFile::dateTimeCallbackCancel();
			StartTimeLog = millis();
			return fileOk;
		}
	}
	return fullCard;
}

bool writeDataToFile() {
	File dataFile = sd.open(LogFile.name, FILE_WRITE);
	if (dataFile) {
		dataFile.print(millis() - StartTimeLog, DEC);
		dataFile.print(',');
		dataFile.print(Sensors.BaroPressure, 1);
		dataFile.print(',');
		dataFile.print(Sensors.BoilerPressure, DEC);
		dataFile.print(',');
		dataFile.print(Settings.BoilerOffset, DEC);
		dataFile.print(',');
		if (Sensors.BoilerType != NoSensor)
			dataFile.print(Sensors.BoilerTemperature,2);
		dataFile.print(',');
		if (Sensors.BoilerABV > 0)
			dataFile.print(Sensors.BoilerABV,1);
		dataFile.print(',');
		dataFile.print(Settings.VaporOffset, DEC);
		dataFile.print(',');
		if (Sensors.VaporType != NoSensor)
			dataFile.print(Sensors.VaporTemperature,2);
		dataFile.print(',');
		if (Sensors.VaporABV > 0)
			dataFile.print(Sensors.VaporABV,1);
		dataFile.print(',');
		if (Sensors.CondenserType != NoSensor)
			dataFile.print(Sensors.CondenserTemperature,2);
		dataFile.println();
		dataFile.close();
		return true;
	}
	return false;
}

void increaseDigit() {
	lcd.noCursor();
	uint8_t value = lcd.read();
	lcd.moveCursorLeft();
	if (value < (9 + 0x30))
		lcd.print(char(++value));
	else
		lcd.print('0');
	lcd.moveCursorLeft();
	lcd.cursor();
}

void decreaseDigit() {
	lcd.noCursor();
	uint8_t value = lcd.read();
	lcd.moveCursorLeft();
	if (value > (0 + 0x30))
		lcd.print(char(--value));
	else
		lcd.print('9');
	lcd.moveCursorLeft();
	lcd.cursor();
}

void nextDigitTime() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	switch (pos) {
	case 9:
	case 12:
	case 9 + 0x40:
	case 12 + 0x40:
		lcd.setCursor(pos + 2, 0);
		break;
	case 15:
		lcd.setCursor(8, 1);
		break;
	case 15 + 0x40:
		lcd.setCursor(8, 0);
		break;
	default:
		lcd.moveCursorRight();
		break;
	}
	lcd.cursor();
}

void prevDigitTime() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	switch (pos) {
	case 11:
	case 14:
	case 11 + 0x40:
	case 14 + 0x40:
		lcd.setCursor(pos - 2, 0);
		break;
	case 8:
		lcd.setCursor(15, 1);
		break;
	case 8 + 0x40:
		lcd.setCursor(15, 0);
		break;
	default:
		lcd.moveCursorLeft();
		break;
	}
	lcd.cursor();
}

void printCancel() {
	lcd.noCursor();
	lcd.clear();
	lcd.printP(msgCanceled);
	delay(1000);
	ReturnPage();
}

void printSave() {
	lcd.noCursor();
	lcd.clear();
	lcd.printP(msgSaved);
	delay(1000);
	ReturnPage();
}

void showMainInit() {
	lcd.clear();
	lcd.noCursor();
	lcd.clearKeys();
	lcd.keyUp.onShortPress = showAlarmsInit;
	lcd.keyDown.onShortPress = showBaroInit;
	lcd.keyRight.onShortPress = nextAlarm;
	lcd.keyRight.onRepPress = nextAlarm;
	lcd.keyLeft.onShortPress = prevAlarm;
	lcd.keyLeft.onRepPress = prevAlarm;
	lcd.keySelect.onShortPress = toggleAlarm;
	lcd.keySelect.onLongPress = toggleSilent;
	showMainRefresh();
	AutoPageFastRefresh = showMainRefreshFast;
	AutoPageRefresh = showMainRefresh;

}


void nextAlarm() {
	if (CurrentAlarm < 4) {
		CurrentAlarm++;
	} else CurrentAlarm = 0;
}

void prevAlarm() {
	if (CurrentAlarm > 0) {
		CurrentAlarm--;
	} else CurrentAlarm = 4;
}

void toggleAlarm() {
	switch (AlarmStatusVapor) {
		case disabled:
			AlarmStatusVapor = armed;
			break;
		case armed:
		case acknowledged:
			AlarmStatusVapor = disabled;
			break;
		case triggered:
			AlarmStatusVapor = acknowledged;
			break;
		default:
			break;
	}
}

void toggleSilent() {
	lcd.clear();
	Silent = !Silent;
	if (Silent)
		lcd.printP(msgSilent);
	else
		lcd.printP(msgAudial);
	delay(1000);
	showMainInit();
}

void printVaporValues() {
	lcd.print(dtostrf(Sensors.VaporTemperature, 6, 2, lineBuffer));
	lcd.print('C');
	lcd.print(' ');
	if (Sensors.VaporABV < ForesAlarm) {
		lcd.printP(msgNoValue);
	}
	else if (Sensors.VaporABV < 0)
		lcd.printP(msgFores);
	else {
		lcd.print(dtostrf(Sensors.VaporABV, 4, 1, lineBuffer));
		lcd.print('%');
	}

	if (CurrentAlarm == 4)
		lcd.printP(msgForesAlarm);
	else
		lcd.print(dtostrf(Settings.Alarm[CurrentAlarm], 3, 0, lineBuffer));
}

void printBoilerValues() {
	lcd.print(dtostrf(Sensors.BoilerTemperature, 6, 2, lineBuffer));
	lcd.print('C');
	lcd.print(' ');
	if (Sensors.BoilerABV < 0)
		lcd.printP(msgNoValue);
	else {
		lcd.print(dtostrf(Sensors.BoilerABV, 4, 1, lineBuffer));
		lcd.print('%');
	}
	for (int i = 0; i < 3; ++i) {
		lcd.print(' ');
	}
}

void printRemainingTime() {
	lcd.print(dtostrf(Sensors.BoilerTemperature, 6, 2, lineBuffer));
	sprintf(lineBuffer, "C %.2hd:%.2hd %.2hd", Sensors.WarmupTime / 60 , Sensors.WarmupTime % 60, Settings.WarmedUp);
	lcd.print(lineBuffer);
}

void showMainRefresh() {
	switch (Sensors.VaporType) {
		case DS18B20:
			lcd.setCursor(0, 0);
			printVaporValues();
			break;
		case NoSensor:
			lcd.setCursor(0, 0);
			lcd.printP(msgNoSensor);
			break;
		default:
			break;
	}
	lcd.setCursor(0, 1);
	switch (Sensors.BoilerType) {
		case DS18B20:
			if (Sensors.WarmupTime > 0)
				printRemainingTime();
			else
				printBoilerValues();
			break;
		case NoSensor:
			lcd.printP(msgNoSensor);
			break;
		default:
			break;
	}
}

void showMainRefreshFast() {
	if (Sensors.VaporType == smt172) {
		lcd.setCursor(0, 0);
		printVaporValues();
	}
}

void showBaroInit() {
	lcd.clear();
	lcd.noCursor();
	lcd.clearKeys();
	lcd.keyUp.onShortPress = showMainInit;
	lcd.keyDown.onShortPress = showTimeInit;
	lcd.keySelect.onLongPress = zeroBoilerPressure;
	ReturnPage = showBaroInit;
	showBaroRefresh();
	AutoPageRefresh = showBaroRefresh;
	AutoPageFastRefresh = nullptr;
}

void zeroBoilerPressure() {
	Settings.BoilerPressureOffset = Sensors.BoilerPressureRaw;
	saveSettings();
	printSave();
}

void showBaroRefresh() {
	lcd.setCursor(0,0);
	lcd.print(dtostrf(Sensors.BaroPressure, 4, 0, lineBuffer));
	lcd.printP(msghPa);
	lcd.print(dtostrf(Sensors.BoilerPressure, 4, 0, lineBuffer));
	lcd.printP(msgdPa);

	lcd.setCursor(0,1);
	lcd.printP(msgBPWater);
	lcd.print(dtostrf(Sensors.H2OBoilingPoint, 6, 2, lineBuffer));
	lcd.moveCursorRight();
	lcd.print('C');
}
void showTimeInit() {
	lcd.clear();
	lcd.noCursor();
	lcd.clearKeys();
	lcd.keyUp.onShortPress = showBaroInit;
	lcd.keyDown.onShortPress = showLogStatusInit;
	lcd.keySelect.onShortPress = setTimeInit;
	showTimeRefresh();
	AutoPageRefresh = showTimeRefresh;
	AutoPageFastRefresh = nullptr;
	ReturnPage = showTimeInit;
}

void showTimeRefresh() {
	rtc.readClock();
	lcd.setCursor(4,0);
	sprintf(lineBuffer, "%.2hd/%.2hd/%.2hd", rtc.year, rtc.month, rtc.day);
	lcd.print(lineBuffer);
	lcd.setCursor(4,1);
	sprintf(lineBuffer, "%.2hd:%.2hd:%.2hd", rtc.hour, rtc.minute, rtc.second);
	lcd.print(lineBuffer);
}

void setTimeInit() {
	AutoPageRefresh = nullptr;
	lcd.clear();
	lcd.noCursor();
	sprintf(lineBuffer, "YYMMDD  %.2hd/%.2hd/%.2hd", rtc.year, rtc.month, rtc.day);
	lcd.print(lineBuffer);
	lcd.setCursor(0,1);
	sprintf(lineBuffer, "HHMMSS  %.2hd:%.2hd:%.2hd", rtc.hour, rtc.minute, rtc.second);
	lcd.print(lineBuffer);
	lcd.setCursor(8,0);
	lcd.cursor();
	lcd.clearKeys();
	lcd.keyUp.onShortPress = increaseDigit;
	lcd.keyUp.onRepPress = increaseDigit;
	lcd.keyDown.onShortPress = decreaseDigit;
	lcd.keyDown.onRepPress = decreaseDigit;
	lcd.keyRight.onShortPress = nextDigitTime;
	lcd.keyRight.onRepPress = nextDigitTime;
	lcd.keyLeft.onShortPress = prevDigitTime;
	lcd.keyLeft.onRepPress = prevDigitTime;
	lcd.keySelect.onShortPress = printCancel;
	lcd.keySelect.onLongPress = setTime;
}

uint8_t lcdGetDoubleDigit() {
	uint8_t value;
	value = (lcd.read() - 0x30) * 10;
	value += (lcd.read() - 0x30);
	return value;
}

void setTime() {
	lcd.noCursor();

	lcd.setCursor(8,0);
	rtc.year = lcdGetDoubleDigit();

	lcd.setCursor(11,0);
	rtc.month = lcdGetDoubleDigit();

	lcd.setCursor(14,0);
	rtc.day = lcdGetDoubleDigit();

	lcd.setCursor(8,1);
	rtc.hour = lcdGetDoubleDigit();

	lcd.setCursor(11,1);
	rtc.minute = lcdGetDoubleDigit();

	lcd.setCursor(14,1);
	rtc.second = lcdGetDoubleDigit();

	rtc.setClock();
	printSave();
}

void showLogStatusInit() {
	lcd.clear();
	lcd.noCursor();
	lcd.clearKeys();
	if (LogFile.isLogging) {
		lcd.setCursor(0,0);
		lcd.print(LogFile.name);
		lcd.setCursor(0,1);
		lcd.printP(msgToStop);
	}
	else {
		lcd.setCursor(0,1);
		lcd.printP(msgToLog);
	}
	lcd.keyUp.onShortPress = showTimeInit;;
	lcd.keyDown.onShortPress = showOffsetVaporInit;
	lcd.keySelect.onShortPress = toggleLogging;
	AutoPageRefresh = nullptr;
	AutoPageFastRefresh = nullptr;
}

void toggleLogging () {
	if (LogFile.isLogging) {
		LogFile.isLogging = false;
		lcd.setCursor(0,0);
		for (int i = 0; i < 11; ++i) {
			lcd.print(' ');
		};
		lcd.setCursor(0,1);
		lcd.printP(msgToLog);
	} else {
		lcd.setCursor(0,0);
		switch (createFile()) {
			case noCard:
				lcd.printP(msgNoCard);
				break;
			case fullCard:
				lcd.printP(msgFullCard);
				break;
			case fileOk:
				lcd.print(LogFile.name);
				LogFile.isLogging = true;
				lcd.setCursor(0,1);
				lcd.printP(msgToStop);
				break;
		}
	}
}

void showOffsetVaporInit() {
	lcd.clear();
	lcd.noCursor();
	lcd.printP(msgVaporOffset);
	printOffsetVapor();
	lcd.setCursor(15,0);
	lcd.print('C');
	lcd.clearKeys();
	lcd.keyUp.onShortPress = showLogStatusInit;
	lcd.keyDown.onShortPress = showOffsetBoilerInit;
	lcd.keySelect.onShortPress = offsetKeyRemap;
	ReturnPage = showOffsetVaporInit;
	AutoPageFastRefresh = showOffsetVaporRefresh;
	oldOffset = Settings.VaporOffset;
	offset = &Settings.VaporOffset;
	PrintOffset = printOffsetVapor;
}


void offsetKeyRemap() {
	lcd.setCursor(14,0);
	lcd.keyRight.onShortPress = nextDigitOffset;
	lcd.keyLeft.onShortPress = prevDigitOffset;
	lcd.keyUp.onShortPress = incDigitOffset;
	lcd.keyUp.onRepPress = incDigitOffset;
	lcd.keyDown.onShortPress = decDigitOffset;
	lcd.keyDown.onRepPress = decDigitOffset;
	lcd.keySelect.onShortPress = offsetCancel;
	lcd.keySelect.onLongPress = offsetSave;
}

void offsetCancel() {
	*offset = oldOffset;
	printCancel();
}

void offsetSave() {
	saveSettings();
	printSave();
}

void printOffsetVapor() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	lcd.setCursor(9,0);
	lcd.print(dtostrf(float(Settings.VaporOffset) / 100, 6, 2, lineBuffer));
	lcd.setCursor(pos,0);
	lcd.cursor();
}

void showOffsetVaporRefresh() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	lcd.setCursor(0,1);
	printVaporValues();
	lcd.setCursor(pos,0);
	lcd.cursor();
}

void showOffsetBoilerInit() {
	lcd.clear();
	lcd.noCursor();
	lcd.clearKeys();
	lcd.printP(msgBoilerOffset);
	printOffsetBoiler();
	lcd.setCursor(15,0);
	lcd.print('C');
	lcd.keyUp.onShortPress = showOffsetVaporInit;
	lcd.keyDown.onShortPress = showAlarmsInit;
	lcd.keySelect.onShortPress = offsetKeyRemap;
	ReturnPage = showOffsetBoilerInit;
	AutoPageRefresh = showOffsetBoilerRefresh;
	AutoPageFastRefresh = nullptr;
	oldOffset = Settings.BoilerOffset;
	offset = &Settings.BoilerOffset;
	PrintOffset = printOffsetBoiler;
}

void printOffsetBoiler() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	lcd.setCursor(9,0);
	lcd.print(dtostrf(float(Settings.BoilerOffset) / 100, 6, 2, lineBuffer));
	lcd.setCursor(pos,0);
	lcd.cursor();
}

void showOffsetBoilerRefresh() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	lcd.setCursor(0,1);
	printBoilerValues();
	lcd.setCursor(pos,0);
	lcd.cursor();
}

void nextDigitOffset() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	switch (pos) {
	case 11:
		lcd.setCursor(13, 0);
		break;
	case 15:
		lcd.setCursor(10, 0);
		break;
	default:
		lcd.moveCursorRight();
		break;
	}
	lcd.cursor();
}

void prevDigitOffset() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	switch (pos) {
	case 13:
		lcd.setCursor(11, 0);
		break;
	case 10:
		lcd.setCursor(15, 0);
		break;
	default:
		lcd.moveCursorLeft();
		break;
	}
	lcd.cursor();
}

void incDigitOffset() {
	uint8_t pos = lcd.getCursor();
	switch (pos) {
	case 10:
		*offset += 1000;
		break;
	case 11:
		*offset += 100;
		break;
	case 13:
		*offset += 10;
		break;
	case 14:
		*offset += 1;
		break;
	case 15:
		*offset = 0;
		break;
	default:
		break;
	}
	*offset = constrain(*offset, -9999, 9999);
	PrintOffset();
}

void decDigitOffset() {
	uint8_t pos = lcd.getCursor();
	switch (pos) {
	case 10:
		*offset -= 1000;
		break;
	case 11:
		*offset -= 100;
		break;
	case 13:
		*offset -= 10;
		break;
	case 14:
		*offset -= 1;
		break;
	case 15:
		*offset = 0;
		break;
	default:
		break;
	}
	*offset = constrain(*offset, -9999, 9999);
	PrintOffset();
}

void showAlarmsInit() {
	lcd.clear();
	lcd.noCursor();
	sprintf(lineBuffer, "1 %.2hd%% 2 %.2hd%%  BLR", Settings.Alarm[0], Settings.Alarm[1]);
	lcd.print(lineBuffer);
	lcd.setCursor(0,1);
	sprintf(lineBuffer, "3 %.2hd%% 4 %.2hd%%  %.2hdC", Settings.Alarm[2], Settings.Alarm[3], Settings.WarmedUp);
	lcd.print(lineBuffer);
	lcd.clearKeys();
	lcd.keyUp.onShortPress = showOffsetBoilerInit;
	lcd.keyDown.onShortPress = showMainInit;
	lcd.keySelect.onShortPress = alarmsKeyRemap;
	ReturnPage =  showAlarmsInit;
	AutoPageRefresh = nullptr;
	AutoPageFastRefresh = nullptr;
}

void alarmsKeyRemap() {
	lcd.setCursor(2,0);
	lcd.cursor();
	lcd.clearKeys();
	lcd.keyUp.onShortPress = increaseDigit;
	lcd.keyUp.onRepPress = increaseDigit;
	lcd.keyDown.onShortPress = decreaseDigit;
	lcd.keyDown.onRepPress = decreaseDigit;
	lcd.keyRight.onShortPress = nextDigitAlarm;
	lcd.keyRight.onRepPress = nextDigitAlarm;
	lcd.keyLeft.onShortPress = prevDigitAlarm;
	lcd.keyLeft.onRepPress = prevDigitAlarm;
	lcd.keySelect.onShortPress = printCancel;
	lcd.keySelect.onLongPress = setAlarms;
}

void nextDigitAlarm() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	switch (pos) {
	case 3:
		lcd.setCursor(8, 0);
		break;
	case 9:
		lcd.setCursor(2, 1);
		break;
	case 3 + 0x40:
		lcd.setCursor(8, 1);
		break;
	case 9 + 0x40:
		lcd.setCursor(13, 1);
		break;
	case 14 + 0x40:
		lcd.setCursor(2, 0);
		break;
	default:
		lcd.moveCursorRight();
		break;
	}
	lcd.cursor();
}

void prevDigitAlarm() {
	uint8_t pos = lcd.getCursor();
	lcd.noCursor();
	switch (pos) {
	case 2:
		lcd.setCursor(14, 1);
		break;
	case 8:
		lcd.setCursor(3, 0);
		break;
	case 2 + 0x40:
		lcd.setCursor(9, 0);
		break;
	case 8 + 0x40:
		lcd.setCursor(3, 1);
		break;
	case 13 + 0x40:
		lcd.setCursor(9, 1);
		break;
	default:
		lcd.moveCursorLeft();
		break;
	}
	lcd.cursor();
}

void setAlarms() {
	lcd.noCursor();

	lcd.setCursor(2,0);
	Settings.Alarm[0] = lcdGetDoubleDigit();

	lcd.setCursor(8,0);
	Settings.Alarm[1] = lcdGetDoubleDigit();

	lcd.setCursor(2,1);
	Settings.Alarm[2] = lcdGetDoubleDigit();

	lcd.setCursor(8,1);
	Settings.Alarm[3] = lcdGetDoubleDigit();

	lcd.setCursor(13,1);
	Settings.WarmedUp = lcdGetDoubleDigit();

	saveSettings();
	printSave();
}

void showBoilerPressureAlarmInit() {
	lcd.clear();
	lcd.noCursor();
	lcd.printP(msgBoilerPressure);
	lcd.clearKeys();
	lcd.keySelect.onShortPress = acknowledgeBoilerAlarm;
	AutoPageRefresh = showBoilerPressureRefresh;
	AutoPageFastRefresh = nullptr;
}

void showBoilerPressureRefresh() {
	lcd.setCursor(4,1);
	lcd.print(dtostrf(Sensors.BoilerPressure, 4, 0, lineBuffer));
	lcd.printP(msgdPa);
}

void showCondenserFailureInit() {
	lcd.clear();
	lcd.noCursor();
	lcd.printP(msgCondenser);
	lcd.clearKeys();
	lcd.keySelect.onShortPress = acknowledgeCondenserAlarm;
	AutoPageRefresh = showCondenserFailureRefresh;
	AutoPageFastRefresh = nullptr;
}

void showCondenserFailureRefresh() {
	lcd.setCursor(0,1);
	if (Sensors.CondenserType == NoSensor)
		lcd.printP(msgNoSensor);
	else {
		lcd.setCursor(3,1);
		lcd.print(dtostrf(Sensors.CondenserTemperature, 6, 2, lineBuffer));
		lcd.print('C');
	}
}

void acknowledgeCondenserAlarm() {
	if (AlarmStatusCondenser == triggered)
		AlarmStatusCondenser = acknowledged;
}

void acknowledgeBoilerAlarm() {
	if (AlarmStatusBoiler == triggered)
		AlarmStatusBoiler = acknowledged;
}












