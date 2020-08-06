/******************************************************************************
 SFE_LSM9DS1.cpp
 SFE_LSM9DS1 Library Source File
 Jim Lindblom @ SparkFun Electronics
 Original Creation Date: February 14, 2014 (Happy Valentines Day!)
 https://github.com/sparkfun/LSM9DS1_Breakout

 This file implements all functions of the LSM9DS1 class. Functions here range
 from higher level stuff, like reading/writing LSM9DS1 registers to low-level,
 hardware reads and writes. Both SPI and I2C handler functions can be found
 towards the bottom of this file.

 Development environment specifics:
 IDE: Arduino 1.0.5
 Hardware Platform: Arduino Pro 3.3V/8MHz
 LSM9DS1 Breakout Version: 1.0

 This code is beerware; if you see me (or any other SparkFun employee) at the
 local, and you've found our code helpful, please buy us a round!

 Distributed as-is; no warranty is given.
 ******************************************************************************/

#include "LSM1MUX.h"
//#include <i2c_t3.h> // Wire library is used for I2C
#include <Wire.h> // Wire library is used for I2C

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifndef I2C_RATE_400
#define I2C_RATE_400 400000
#endif

//#define MAX_MUX 16
//MUX settings
//#define S0 12
//#define S1 11
//#define S2 10
//#define S3 9

LSM9DS1::LSM9DS1(interface_mode interface, uint8_t xgAddr, uint8_t mAddr) {
	// interfaceMode will keep track of whether we're using SPI or I2C:
	interfaceMode = interface;

	// xmAddress and xgAddress will store the 7-bit I2C address, if using I2C.
	// If we're using SPI, these variables store the chip-select pins.
	mAddress = mAddr;
	xgAddress = xgAddr;
  
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  
}

uint16_t LSM9DS1::begin(gyro_scale gScl, accel_scale aScl, mag_scale mScl,
		gyro_odr gODR, accel_odr aODR, mag_odr mODR) {
	// Store the given scales in class variables. These scale variables
	// are used throughout to calculate the actual g's, DPS,and Gs's.
	gScale = gScl;
	aScale = aScl;
	mScale = mScl;

	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	calcgRes(); // Calculate DPS / ADC tick, stored in gRes variable
	calcmRes(); // Calculate Gs / ADC tick, stored in mRes variable
	calcaRes(); // Calculate g / ADC tick, stored in aRes variable

	initI2C();					// Initialize I2C

	// To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
	uint8_t xgTest = xgReadByte(WHO_AM_I_XG);		// Read the accel/gyro WHO_AM_I
	uint8_t mTest = mReadByte(WHO_AM_I_M);	// Read the mag WHO_AM_I

	// Gyro and Accelerometer initialization stuff:
	setAccelODR(aODR); // Set the accel data rate.
	setAccelScale (aScale); // Set the accel range.
  initGyroAccel(); // This will "turn on" the gyro and accel. Setting up interrupts, etc.	
	setGyroAccelODR(gODR); // Set the gyro output data rate and bandwidth.
	setGyroScale (gScale); // Set the gyro range
	

	// Magnetometer initialization stuff:
	initMag(); // "Turn on" all axes of the mag. Set up interrupts, etc.
	setMagODR(mODR); // Set the magnetometer output data rate.
	setMagScale (mScale); // Set the magnetometer's range.

	// Once everything is initialized, return the WHO_AM_I registers we read:
	return (mTest << 8) | xgTest;
}

uint16_t LSM9DS1::beginID(short sensorID, gyro_scale gScl, accel_scale aScl, mag_scale mScl,
		gyro_odr gODR, accel_odr aODR, mag_odr mODR) {
	readMux(sensorID);
	delay(1);
	return begin(gScl, aScl, mScl,	gODR, aODR, mODR);
}

void LSM9DS1::initGyroAccel() {
	xgWriteByte(CTRL_REG4, 0x3F); // Normal mode, gyro x/y/z all enabled 
	xgWriteByte(CTRL_REG5_XL, 0x38); // accel x/y/z all enabled

	xgWriteByte(CTRL_REG3_G, 0x00); // Normal mode, high cutoff frequency

	xgWriteByte(INT1_CTRL, 0x82); // gyro: INTERRUPT ENABLE and DATA READY ON INT1_A/G
	xgWriteByte(INT2_CTRL, 0x01); // accel: data ready on INT2_A/G 

	xgWriteByte(CTRL_REG6_XL, 0x40); // ACCEL 50Hz data rate, set scale to 2g
	xgWriteByte(CTRL_REG1_G, 0x20); // GYRO Set scale to 245 dps

	xgWriteByte(CTRL_REG9, 0x00); // check

	// Temporary !!! For testing !!! Remove !!! Or make useful !!!
	//configGyroInt(0x2A, 0, 0, 0, 0); // Trigger interrupt when above 0 DPS...
}

void LSM9DS1::initMag() {

	mWriteByte(CTRL_REG1_M, 0x9C); // Mag data rate - 80 Hz, enable temperature compensation 

	mWriteByte(CTRL_REG2_M, 0x00); // Mag scale to +/- 4GS

	mWriteByte(CTRL_REG3_M, 0x00); // Continuous conversion mode

	mWriteByte(INT_CFG_M, 0x05); // Enable interrupts for mag, active-high, push-pull
}

void LSM9DS1::calLSM9DS1(short sensorID) { //TODO: change to array
	readMux(sensorID);
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	int16_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
	int samples = 0, ii;

	// First get gyro bias
	byte c = xgReadByte(CTRL_REG9);
	xgWriteByte(CTRL_REG9, c | 0x02);         // Enable gyro FIFO  
	delay(20);                                 // Wait for change to take effect
	xgWriteByte(FIFO_CTRL, 0x20 | 0x1F); // Enable gyro FIFO stream mode and set watermark at 32 samples
	delay(1000);  // delay 1000 milliseconds to collect FIFO samples

	samples = (xgReadByte(FIFO_SRC) & 0x1F); // Read number of stored samples

	for (ii = 0; ii < samples; ii++) {  // Read the gyro data stored in the FIFO
		xgReadBytes(OUT_X_L_G, &data[0], 6);
		gyro_bias[0] += (((int16_t) data[1] << 8) | data[0]);
		gyro_bias[1] += (((int16_t) data[3] << 8) | data[2]);
		gyro_bias[2] += (((int16_t) data[5] << 8) | data[4]);
	}

    //gyro_bias[0] /= samples; // average the data
	//gyro_bias[1] /= samples;
	//gyro_bias[2] /= samples;
    gbias[sensorID][0] = (float) (gyro_bias[0] / samples) * gRes; // Properly scale the data to get deg/s
	gbias[sensorID][1] = (float) (gyro_bias[1] / samples) * gRes;
	gbias[sensorID][2] = (float) (gyro_bias[2] / samples) * gRes;
	//gbias[sensorID][0] = (float) gyro_bias[0] * gRes; // Properly scale the data to get deg/s
	//gbias[sensorID][1] = (float) gyro_bias[1] * gRes;
	//gbias[sensorID][2] = (float) gyro_bias[2] * gRes;

	c = xgReadByte(CTRL_REG9);
	xgWriteByte(CTRL_REG9, c & ~0x02);  // Disable gyro FIFO  
	delay(20);
	xgWriteByte(FIFO_CTRL, 0x00);   // Enable gyro bypass mode

	//  Now get the accelerometer biases
	c = xgReadByte(CTRL_REG9);
	xgWriteByte(CTRL_REG9, c | 0x02);      // Enable accelerometer FIFO  
	delay(20);                                // Wait for change to take effect
	xgWriteByte(FIFO_CTRL, 0x20 | 0x1F); // Enable accelerometer FIFO stream mode and set watermark at 32 samples
	delay(1000);  // delay 1000 milliseconds to collect FIFO samples

	samples = (xgReadByte(FIFO_SRC) & 0x1F); // Read number of stored accelerometer samples

	for (ii = 0; ii < samples; ii++) { // Read the accelerometer data stored in the FIFO
		xgReadBytes(OUT_X_L_A_XL, &data[0], 6);
		accel_bias[0] += (((int16_t) data[1] << 8) | data[0]);
		accel_bias[1] += (((int16_t) data[3] << 8) | data[2]);
		accel_bias[2] += (((int16_t) data[5] << 8) | data[4])
				- (int16_t)(1. / aRes); // Assumes sensor facing up!
	}

	accel_bias[0] /= samples; // average the data
	accel_bias[1] /= samples;
	accel_bias[2] /= samples;

	abias[sensorID][0] = (float) accel_bias[0] * aRes; // Properly scale data to get gs
	abias[sensorID][1] = (float) accel_bias[1] * aRes;
	abias[sensorID][2] = (float) accel_bias[2] * aRes;

	c = xgReadByte(CTRL_REG9);
	xgWriteByte(CTRL_REG9, c & ~0x40);    // Disable accelerometer FIFO  
	delay(20);
	xgWriteByte(FIFO_CTRL, 0x00);       // Enable accelerometer bypass mode
}

void LSM9DS1::readAccel(short sensorID) { //TODO: change to array
	uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp	
	xgReadBytes(OUT_X_L_A_XL, temp, 6); // Read 6 bytes, beginning at OUT_X_L_A_XL
	ax[sensorID] = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
	ay[sensorID] = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
	az[sensorID] = (temp[5] << 8) | temp[4]; // Store z-axis values into az
}

void LSM9DS1::readMag(short sensorID) { //TODOgbias: change to array
	uint8_t temp[6]; // We'll read six bytes from the mag into temp	
	mReadBytes(OUT_X_L_M, temp, 6); // Read 6 bytes, beginning at OUT_X_L_M
	mx[sensorID] = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
	my[sensorID] = (temp[3] << 8) | temp[2]; // Store y-axis values into my
	mz[sensorID] = (temp[5] << 8) | temp[4]; // Store z-axis values into mz
}

void LSM9DS1::readTemp(short sensorID) { //TODO: change to array
	uint8_t temp[2]; // We'll read two bytes from the temperature sensor into temp	
	mReadBytes(OUT_TEMP_L, temp, 2); // Read 2 bytes, beginning at OUT_TEMP_L_M
	temperature[sensorID] = (((int16_t) temp[1] << 12) | temp[0] << 4) >> 4; // Temperature is a 12-bit signed integer
}

void LSM9DS1::readGyro(short sensorID) { //TODO: change to array
	uint8_t temp[6]; // We'll read six bytes from the gyro into temp
	xgReadBytes(OUT_X_L_G, temp, 6); // Read 6 bytes, beginning at OUT_X_L_G
	gx[sensorID] = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
	gy[sensorID] = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
	gz[sensorID] = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
}

float LSM9DS1::calcGyro(int16_t gyro) {
	// Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
	return gRes * gyro;
}

float LSM9DS1::calcAccel(int16_t accel) {
	// Return the accel raw reading times our pre-calculated g's / (ADC tick):
	return aRes * accel;
}

float LSM9DS1::calcMag(int16_t mag) {
	// Return the mag raw reading times our pre-calculated Gs / (ADC tick):
	return mRes * mag;
}

void LSM9DS1::setGyroScale(gyro_scale gScl) {
	// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
	uint8_t temp = xgReadByte(CTRL_REG1_G);
	// Then mask out the gyro scale bits:
	temp &= 0xFF ^ (0x3 << 3);
	// Then shift in our new scale bits:
	temp |= gScl << 3;
	// And write the new register value back into CTRL_REG1_G:
	xgWriteByte(CTRL_REG1_G, temp);

	// We've updated the sensor, but we also need to update our class variables
	// First update gScale:
	gScale = gScl;
	// Then calculate a new gRes, which relies on gScale being set correctly:
	calcgRes();
}

void LSM9DS1::setAccelScale(accel_scale aScl) {
	// We need to preserve the other bytes in CTRL_REG6_XL. So, first read it:
	uint8_t temp = xgReadByte(CTRL_REG6_XL);
	// Then mask out the accel scale bits:
	temp &= 0xFF ^ (0x3 << 3);
	// Then shift in our new scale bits:
	temp |= aScl << 3;
	// And write the new register value back into CTRL_REG6_XL:
	xgWriteByte(CTRL_REG6_XL, temp);

	// We've updated the sensor, but we also need to update our class variables
	// First update aScale:
	aScale = aScl;
	// Then calculate a new aRes, which relies on aScale being set correctly:
	calcaRes();
}

void LSM9DS1::setMagScale(mag_scale mScl) {
	// We need to preserve the other bytes in CTRL_REG2_M. So, first read it:
	uint8_t temp = mReadByte(CTRL_REG2_M);
	// Then mask out the mag scale bits:
	temp &= 0xFF ^ (0x3 << 5);
	// Then shift in our new scale bits:
	temp |= mScl << 5;
	// And write the new register value back into CTRL_REG2_M:
	mWriteByte(CTRL_REG2_M, temp);

	// We've updated the sensor, but we also need to update our class variables
	// First update mScale:
	mScale = mScl;
	// Then calculate a new mRes, which relies on mScale being set correctly:
	calcmRes();
}

void LSM9DS1::setGyroAccelODR(gyro_odr gRate) {
	uint8_t gRate_bw; 
	gRate_bw = gRate; 
	gRate_bw &= 0x03;
	uint8_t	gRate_odr; 
	gRate_odr = gRate; 
	gRate_odr &= 0x1C;
  gRate_odr = gRate_odr >> 2;
	//saving ODR:
	// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
	uint8_t temp = xgReadByte(CTRL_REG1_G);
	// Then mask out the gyro ODR bits:
	temp &= 0xFF ^ (0x7 << 5);
	// Then shift in our new ODR bits:
	temp |= (gRate_odr << 5);
	//saving BW:
	temp &= 0xFF ^ (0x3);
	// Then shift in our new BW bits:
	temp |= (gRate_bw);
	// And write the new register value back into CTRL_REG1_G:
	xgWriteByte(CTRL_REG1_G, temp);
}

/*/TEST FUNCTION
void LSM9DS1::setGyroAccelODR(gyro_odr gRate) {//Temporary function if tests fail for using GYRO values.
	uint8_t gRate_bw &= 0x02
	uint8_t	gRate_odr &= 0x1C
	//saving ODR:
	// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
	uint8_t temp = xgReadByte(CTRL_REG1_G);
	// Then mask out the gyro ODR bits:
	temp &= 0xFF ^ (0x7 << 5);
	// Then shift in our new ODR bits:
	temp |= (gRate_odr << 5);
	//saving BW:
	temp &= 0xFF ^ (0x3);
	// Then shift in our new BW bits:
	temp |= (gRate_bw);
	// And write the new register value back into CTRL_REG1_G:
	xgWriteByte(CTRL_REG1_G, temp);
}
*/ 

void LSM9DS1::setAccelODR(accel_odr aRate) {
	// We need to preserve the other bytes in CTRL_REG6_XL. So, first read it:
	uint8_t temp = xgReadByte(CTRL_REG6_XL);
	// Then mask out the accel ODR bits:
	temp &= 0xFF ^ (0x7 << 5);
	// Then shift in our new ODR bits:
	temp |= (aRate << 5);
	// And write the new register value back into CTRL_REG6_XL:
	xgWriteByte(CTRL_REG6_XL, temp);
}

void LSM9DS1::setAccelABW(accel_abw abwRate) {
	// We need to preserve the other bytes in CTRL_REG6_XL. So, first read it:
	uint8_t temp = xgReadByte(CTRL_REG6_XL);
	// Then mask out the accel ABW bits:
	temp &= 0xFF ^ (0x3);
	// Then shift in our new ODR bits:
	temp |= (abwRate);
	// And write the new register value back into CTRL_REG6_XL:
	xgWriteByte(CTRL_REG6_XL, temp);
}

void LSM9DS1::setMagODR(mag_odr mRate) {
	// We need to preserve the other bytes in CTRL_REG1_M. So, first read it:
	uint8_t temp = mReadByte(CTRL_REG1_M);
	// Then mask out the mag ODR bits:
	temp &= 0xFF ^ (0x7 << 2);
	// Then shift in our new ODR bits:
	temp |= (mRate << 2);
	// And write the new register value back into CTRL_REG1_M:
	mWriteByte(CTRL_REG1_M, temp);
}

void LSM9DS1::configGyroInt(uint8_t int1Cfg, uint16_t int1ThsX,
		uint16_t int1ThsY, uint16_t int1ThsZ, uint8_t duration) {
	xgWriteByte(INT1_CFG_G, int1Cfg);
	xgWriteByte(INT1_THS_XH_G, (int1ThsX & 0xFF00) >> 8);
	xgWriteByte(INT1_THS_XL_G, (int1ThsX & 0xFF));
	xgWriteByte(INT1_THS_YH_G, (int1ThsY & 0xFF00) >> 8);
	xgWriteByte(INT1_THS_YL_G, (int1ThsY & 0xFF));
	xgWriteByte(INT1_THS_ZH_G, (int1ThsZ & 0xFF00) >> 8);
	xgWriteByte(INT1_THS_ZL_G, (int1ThsZ & 0xFF));
	if (duration)
		xgWriteByte(INT1_DURATION_G, 0x80 | duration);
	else
		xgWriteByte(INT1_DURATION_G, 0x00);
}

void LSM9DS1::calcgRes() {
	switch (gScale) {
	case G_SCALE_245DPS:
		gRes = 245.0 / 32768.0;
		break;
	case G_SCALE_500DPS:
		gRes = 500.0 / 32768.0;
		break;
  case G_SCALE_1000DPS_NA: //not available
    gRes = 1000.0 / 32768.0;
    break;
	case G_SCALE_2000DPS:
		gRes = 2000.0 / 32768.0;
		break;
	}
}

void LSM9DS1::calcaRes() {
	switch (aScale){
	case A_SCALE_2G:
		aRes = 2.0 / 32768.0;
		break;
	case A_SCALE_16G:
		aRes = 16.0 / 32768.0;
		break;
	case A_SCALE_4G:
		aRes = 4.0 / 32768.0;
		break;
	case A_SCALE_8G:
		aRes = 8.0 / 32768.0;
		break;
	}
}

void LSM9DS1::calcmRes() {

	switch (mScale){
	case M_SCALE_4GS:
		mRes = 4.0 / 32768.0;
		break;
	case M_SCALE_8GS:
		mRes = 8.0 / 32768.0;
		break;
	case M_SCALE_12GS:
		mRes = 12.0 / 32768.0;
		break;
	case M_SCALE_16GS:
		mRes = 16.0 / 32768.0;
		break;
	}
}

void LSM9DS1::xgWriteByte(uint8_t subAddress, uint8_t data) {

	I2CwriteByte(xgAddress, subAddress, data);

}

void LSM9DS1::mWriteByte(uint8_t subAddress, uint8_t data) {

	return I2CwriteByte(mAddress, subAddress, data);

}

uint8_t LSM9DS1::xgReadByte(uint8_t subAddress) {

	return I2CreadByte(xgAddress, subAddress);

}

void LSM9DS1::xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count) {

	I2CreadBytes(xgAddress, subAddress, dest, count);

}

uint8_t LSM9DS1::mReadByte(uint8_t subAddress) {

	return I2CreadByte(mAddress, subAddress);

}

void LSM9DS1::mReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count) {

	I2CreadBytes(mAddress, subAddress, dest, count);

}

void LSM9DS1::initI2C() {
	Wire.begin();	// Initialize I2C library
	//Wire.setRate(I2C_RATE_400);
    Wire.setClock(I2C_RATE_400);
}

// Wire.h read and write protocols
void LSM9DS1::I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data) {
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

uint8_t LSM9DS1::I2CreadByte(uint8_t address, uint8_t subAddress) {
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	          // Put slave register address in Tx buffer
	Wire.endTransmission(false); // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, (uint8_t) 1); // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                         // Return data read from slave register
}

void LSM9DS1::I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest,
		uint8_t count) {
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	// Next send the register to be read. OR with 0x80 to indicate multi-read.
	Wire.write(subAddress | 0x80);    // Put slave register address in Tx buffer
	Wire.endTransmission(false); // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
	Wire.requestFrom(address, count); // Read bytes from slave register address 
	while (Wire.available()) {
		dest[i++] = Wire.read(); // Put read results in the Rx buffer
	}
}

void LSM9DS1::readMux(short channel) {
	int controlPin[] = { S0, S1, S2, S3 };
	int muxChannel[16][4] = {
			{ 0, 0, 0, 0 }, //channel 0
			{ 1, 0, 0, 0 },
      { 0, 1, 0, 0 },
      { 1, 1, 0, 0 },
      { 0, 0, 1, 0 },
      { 1, 0, 1, 0 },
      { 0, 1, 1, 0 },
      { 1, 1, 1, 0 },
      { 0, 0, 0, 1 },
      { 1, 0, 0, 1 },
      { 0, 1, 0, 1 },
      { 1, 1, 0, 1 },
			{ 0, 0, 1, 1 },
      { 1, 0, 1, 1 },
      { 0, 1, 1, 1 },
      { 1, 1, 1, 1 } //channel 15
	};
	for (int i = 0; i < 4; i++) {
		digitalWrite(controlPin[i], muxChannel[channel][i]);
	}
}
