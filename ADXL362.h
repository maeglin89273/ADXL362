/*
 Arduino Library for Analog Devices ADXL362 - Micropower 3-axis accelerometer
 go to http://www.analog.com/ADXL362 for datasheet
 
 
 License: CC BY-SA 3.0: Creative Commons Share-alike 3.0. Feel free 
 to use and abuse this code however you'd like. If you find it useful
 please attribute, and SHARE-ALIKE!
 
 Created June 2012
 by Anne Mahaffey - hosted on http://annem.github.com/ADXL362
 Modified Mars 2014
 by pixelk
 
 */ 

#include "Arduino.h"

#ifndef ADXL362_h
#define ADXL362_h


/**
 * Turn debugging on or off
 */
//#define ADXL362_DEBUG 1

/* ADXL Registers */

#define XL362_DEVID_AD			0x00
#define XL362_DEVID_MST			0x01
#define XL362_PARTID			0x02
#define XL362_REVID			0x03
#define XL362_XDATA			0x08
#define XL362_YDATA			0x09
#define XL362_ZDATA			0x0A
#define XL362_STATUS			0x0B
#define XL362_FIFO_ENTRIES_L		0x0C
#define XL362_FIFO_ENTRIES_H		0x0D
#define XL362_XDATA_L			0x0E
#define XL362_XDATA_H			0x0F
#define XL362_YDATA_L			0x10
#define XL362_YDATA_H			0x11
#define XL362_ZDATA_L			0x12
#define XL362_ZDATA_H			0x13
#define XL362_TEMP_L			0x14
#define XL362_TEMP_H			0x15
#define XL362_SOFT_RESET		0x1F
#define XL362_THRESH_ACT_L		0x20
#define XL362_THRESH_ACT_H		0x21
#define XL362_TIME_ACT				0x22
#define XL362_THRESH_INACT_L	0x23
#define XL362_THRESH_INACT_H	0x24
#define XL362_TIME_INACT_L		0x25
#define XL362_TIME_INACT_H		0x26
#define XL362_ACT_INACT_CTL		0x27
#define XL362_FIFO_CONTROL		0x28
#define XL362_FIFO_SAMPLES		0x29
#define XL362_INTMAP1			0x2A
#define XL362_INTMAP2			0x2B
#define XL362_FILTER_CTL		0x2C
#define XL362_POWER_CTL			0x2D
#define XL362_SELF_TEST			0x2E

/* Configuration values */

#define XL362_FILTER_FLAG_2G B00000000
#define XL362_FILTER_FLAG_4G B01000000
#define XL362_FILTER_FLAG_8G B10000000

#define XL362_FILTER_FLAG_HBW B10000
#define XL362_FILTER_FLAG_FBW B00000

#define XL362_FILTER_FLAG_ODR12  B000
#define XL362_FILTER_FLAG_ODR25  B001
#define XL362_FILTER_FLAG_ODR50  B010
#define XL362_FILTER_FLAG_ODR100 B011
#define XL362_FILTER_FLAG_ODR200 B100
#define XL362_FILTER_FLAG_ODR400 B111

#define XL362_POWER_FLAG_NOISE_NORMAL   B000000
#define XL362_POWER_FLAG_NOISE_LOW      B010000
#define XL362_POWER_FLAG_NOISE_ULTRALOW B100000

#define XL362_POWER_FLAG_MEASURE_STANDBY B00
#define XL362_POWER_FLAG_MEASURE_RUNING  B10

class ADXL362
{
public:

	ADXL362(uint8_t slaveSelectPin = 10);
	
	//
	// Basic Device control and readback functions
	//
	void begin(); 		
	void beginMeasure(); 
	int readXData();
	int readYData();
	int readZData();
	void readXYZTData(short &XData, short &YData, short &ZData, float &Temperature);
	void readXYZmg(int &X, int &Y, int &Z);
	void XYZmgtoRPT(int X, int Y, int Z, float &Rho, float &Phi, float &Theta);
	int readTemp();
	
	//
	// Activity/Inactivity interrupt functions
	//
	void setupDCActivityInterrupt(int threshold, uint8_t time);	
	void setupDCInactivityInterrupt(int threshold, uint8_t time);
	void setupACActivityInterrupt(int threshold, uint8_t time);
	void setupACInactivityInterrupt(int threshold, uint8_t time);
	
	// need to add the following functions
	// void mapINT1(
	// void mapINT2
	// void autoSleep
	// void activityInterruptControl
	//		-Activity, Inactivity, Both
	//		- Referenced, Absolute
	//		- Free Fall, Linked Mode, Loop Mode
	
	
	void checkAllControlRegs();
	
	void setRange(uint8_t Range);
	void setBandwidth(uint8_t BandWidth);
	void setOutputDatarate(uint8_t ODR);
	void setNoiseLevel(uint8_t NoiseLevel);
	
	//  Low-level SPI control, to simplify overall coding
	uint8_t SPIreadOneRegister(uint8_t regAddress);
	void SPIwriteOneRegister(uint8_t regAddress, uint8_t regValue);
	int  SPIreadTwoRegisters(uint8_t regAddress);
	void SPIwriteTwoRegisters(uint8_t regAddress, int twoRegValue);
	
private:
	const uint8_t slaveSelectPin;
	uint8_t mgperLSB; // default +-2g XL362_FILTER_FLAG_2G -> 1mg/LSB (ADXL362 Datasheet page 4)

};

#endif
