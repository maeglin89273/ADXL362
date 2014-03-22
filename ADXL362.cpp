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

#include <Arduino.h>
#include <ADXL362.h>
#include <SPI.h>

const int slaveSelectPin = 10;

ADXL362::ADXL362() {

}


//
//  begin()
//  Initial SPI setup, soft reset of device
//
void ADXL362::begin() {
  pinMode(slaveSelectPin, OUTPUT);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);	//CPHA = CPOL = 0    MODE = 0
  delay(1000);
    
  // soft reset
  mgperLSB = 1;
  SPIwriteOneRegister(XL362_SOFT_RESET, 0x52);  // Write to SOFT RESET, "R"
  delay(10);
#ifdef ADXL362_DEBUG 
  Serial.println("Soft Reset\n");
#endif
 }

 
//
//  beginMeasure()
//  turn on Measurement mode - required after reset
// 
void ADXL362::beginMeasure() {
  uint8_t temp = SPIreadOneRegister(XL362_POWER_CTL);	// read Reg 2D before modifying for measure mode
#ifdef ADXL362_DEBUG
  Serial.print(  "Setting Measeurement Mode - Reg XL362_POWER_CTL before = "); 
  Serial.print(temp);
#endif

  // turn on measurement mode
  temp = (temp & B11111100) | XL362_POWER_FLAG_MEASURE_RUNING;			// turn on measurement bit in Reg XL362_POWER_CTL
  SPIwriteOneRegister(XL362_POWER_CTL, temp); // Write to XL362_POWER_CTL, Measurement Mode
  delay(10);	
  
#ifdef ADXL362_DEBUG
  temp = SPIreadOneRegister(XL362_POWER_CTL);
  Serial.print(  ", Reg XL362_POWER_CTL after = "); 
  Serial.println(temp); 
#endif
}

//
//  readXData(), readYData(), readZData(), readTemp()
//  Read X, Y, Z, and Temp registers
//
int ADXL362::readXData(){
  int XDATA = SPIreadTwoRegisters(XL362_XDATA_L);
#ifdef ADXL362_DEBUG
  Serial.print(  "XDATA = "); 
  Serial.println(XDATA);
#endif
  return XDATA;
}

int ADXL362::readYData(){
  int YDATA = SPIreadTwoRegisters(XL362_YDATA_L);
#ifdef ADXL362_DEBUG
  Serial.print(  "\tYDATA = "); 
  Serial.println(YDATA);
#endif
  return YDATA;
}

int ADXL362::readZData(){
  int ZDATA = SPIreadTwoRegisters(XL362_ZDATA_L);
#ifdef ADXL362_DEBUG
  Serial.print(  "\tZDATA = "); 
  Serial.println(ZDATA);
#endif
  return ZDATA;
}

int ADXL362::readTemp(){
  int TEMP = SPIreadTwoRegisters(XL362_TEMP_L);
#ifdef ADXL362_DEBUG
  Serial.print("\tTEMP = "); 
  Serial.println(TEMP);
#endif
  return TEMP;
}

void ADXL362::readXYZTData(short &XData, short &YData, short &ZData, float &Temperature){
  
  // burst SPI read
  // A burst read of all three axis is required to guarantee all measurements correspond to same sample time
  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(0x0B);  // read instruction
  SPI.transfer(XL362_XDATA_L);  // Start at XData Reg
  XData = SPI.transfer(0x00);
  XData = XData + ((short)SPI.transfer(0x00) << 8);
  YData = SPI.transfer(0x00);
  YData = YData + ((short)SPI.transfer(0x00) << 8);
  ZData = SPI.transfer(0x00);
  ZData = ZData + ((short)SPI.transfer(0x00) << 8);
  short RawTemperature = SPI.transfer(0x00);
  RawTemperature = RawTemperature + ((short)SPI.transfer(0x00) << 8);
  Temperature = (float)RawTemperature * 0.065;
  digitalWrite(slaveSelectPin, HIGH);
  
#ifdef ADXL362_DEBUG
  Serial.print(  "XDATA = "); Serial.print(XData); 
  Serial.print(  "\tYDATA = "); Serial.print(YData); 
  Serial.print(  "\tZDATA = "); Serial.print(ZData); 
  Serial.println(  "\tTemperature = "); Serial.println(Temperature);
#endif
}

void ADXL362::readXYZmg(int &X, int &Y, int &Z){
  // burst SPI read
  // A burst read of all three axis is required to guarantee all measurements correspond to same sample time
  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(0x0B);  // read instruction
  SPI.transfer(XL362_XDATA_L);  // Start at XData Reg
  short XData = SPI.transfer(0x00);
  XData = XData + ((short)SPI.transfer(0x00) << 8);
  short YData = SPI.transfer(0x00);
  YData = YData + ((short)SPI.transfer(0x00) << 8);
  short ZData = SPI.transfer(0x00);
  ZData = ZData + ((short)SPI.transfer(0x00) << 8);
  digitalWrite(slaveSelectPin, HIGH);
  
  X = (int)XData * mgperLSB;
  Y = (int)YData * mgperLSB;
  Z = (int)ZData * mgperLSB;
  
#ifdef ADXL362_DEBUG
  Serial.print(  "x = "); Serial.print(X); 
  Serial.print(  "\ty = "); Serial.print(Y); 
  Serial.println(  "\tz = "); Serial.print(Z); 
#endif
}

void ADXL362::XYZmgtoRPT(int X, int Y, int Z, float &Rho, float &Phi, float &Theta){
	Rho = atan2(float(X), sqrt(pow(float(Y),2)+pow(float(Z),2)));
  Rho *= 180/M_PI; 

  Phi = atan2(float(Y), sqrt(pow(float(X),2)+pow(float(Z),2)));
  Phi *= 180/M_PI; 

  Theta = atan2(sqrt(pow(float(X),2)+pow(float(Y),2)),float(Z));
  Theta *= 180/M_PI; 	
}

void ADXL362::setupDCActivityInterrupt(int threshold, uint8_t time){
  //  Setup motion and time thresholds
  SPIwriteTwoRegisters(XL362_THRESH_ACT_L, threshold);
  SPIwriteOneRegister(XL362_TIME_ACT, time);

  // turn on activity interrupt
  uint8_t ACT_INACT_CTL_Reg = SPIreadOneRegister(XL362_ACT_INACT_CTL);  // Read current reg value
  ACT_INACT_CTL_Reg = ACT_INACT_CTL_Reg | (0x01);     // turn on bit 1, ACT_EN  
  SPIwriteOneRegister(XL362_ACT_INACT_CTL, ACT_INACT_CTL_Reg);       // Write new reg value 

#ifdef ADXL362_DEBUG
  Serial.print("DC Activity Threshold set to "); Serial.print(SPIreadTwoRegisters(XL362_THRESH_ACT_L));
  Serial.print(", Time threshold set to ");  		 Serial.print(SPIreadOneRegister(XL362_TIME_ACT)); 
  Serial.print(", ACT_INACT_CTL Register is ");  Serial.println(SPIreadOneRegister(XL362_ACT_INACT_CTL), HEX);
#endif
}

void ADXL362::setupACActivityInterrupt(int threshold, uint8_t time){
  //  Setup motion and time thresholds
  SPIwriteTwoRegisters(XL362_THRESH_ACT_L, threshold);
  SPIwriteOneRegister(XL362_TIME_ACT, time);
  
  // turn on activity interrupt
  uint8_t ACT_INACT_CTL_Reg = SPIreadOneRegister(XL362_ACT_INACT_CTL);  // Read current reg value
  ACT_INACT_CTL_Reg = ACT_INACT_CTL_Reg | (0x03);     // turn on bit 2 and 1, ACT_AC_DCB, ACT_EN  
  SPIwriteOneRegister(XL362_ACT_INACT_CTL, ACT_INACT_CTL_Reg);       // Write new reg value 

#ifdef ADXL362_DEBUG
  Serial.print("AC Activity Threshold set to "); Serial.print(SPIreadTwoRegisters(XL362_THRESH_ACT_L));
  Serial.print(", Time threshold set to ");  		 Serial.print(SPIreadOneRegister(XL362_TIME_ACT)); 
  Serial.print(", ACT_INACT_CTL Register is ");  Serial.println(SPIreadOneRegister(XL362_ACT_INACT_CTL), HEX);
#endif
}

void ADXL362::setupDCInactivityInterrupt(int threshold, uint8_t time){
  //  Setup motion and time thresholds
  SPIwriteTwoRegisters(XL362_THRESH_ACT_L, threshold);
  SPIwriteOneRegister(XL362_TIME_ACT, time);

  // turn on inactivity interrupt
  uint8_t ACT_INACT_CTL_Reg = SPIreadOneRegister(XL362_ACT_INACT_CTL);   // Read current reg value 
  ACT_INACT_CTL_Reg = ACT_INACT_CTL_Reg | (0x04);      // turn on bit 3, INACT_EN  
  SPIwriteOneRegister(XL362_ACT_INACT_CTL, ACT_INACT_CTL_Reg);       // Write new reg value 

#ifdef ADXL362_DEBUG
  Serial.print("DC Activity Threshold set to "); Serial.print(SPIreadTwoRegisters(XL362_THRESH_ACT_L));
  Serial.print(", Time threshold set to ");  		 Serial.print(SPIreadOneRegister(XL362_TIME_ACT)); 
  Serial.print(", ACT_INACT_CTL Register is ");  Serial.println(SPIreadOneRegister(XL362_ACT_INACT_CTL), HEX);
#endif
}


void ADXL362::setupACInactivityInterrupt(int threshold, uint8_t time){
  //  Setup motion and time thresholds
  SPIwriteTwoRegisters(XL362_THRESH_ACT_L, threshold);
  SPIwriteOneRegister(XL362_TIME_ACT, time);
 
  // turn on inactivity interrupt
  uint8_t ACT_INACT_CTL_Reg = SPIreadOneRegister(XL362_ACT_INACT_CTL);   // Read current reg value
  ACT_INACT_CTL_Reg = ACT_INACT_CTL_Reg | (0x0C);      // turn on bit 3 and 4, INACT_AC_DCB, INACT_EN  
  SPIwriteOneRegister(0x27, ACT_INACT_CTL_Reg);        // Write new reg value 
  SPIwriteOneRegister(XL362_ACT_INACT_CTL, ACT_INACT_CTL_Reg);       // Write new reg value 

#ifdef ADXL362_DEBUG
  Serial.print("AC Activity Threshold set to "); Serial.print(SPIreadTwoRegisters(XL362_THRESH_ACT_L));
  Serial.print(", Time threshold set to ");  		 Serial.print(SPIreadOneRegister(XL362_TIME_ACT)); 
  Serial.print(", ACT_INACT_CTL Register is ");  Serial.println(SPIreadOneRegister(XL362_ACT_INACT_CTL), HEX);
#endif
}


void ADXL362::checkAllControlRegs(){
	//uint8_t filterCntlReg = SPIreadOneRegister(0x2C);
	//uint8_t ODR = filterCntlReg & 0x07;  Serial.print("ODR = ");  Serial.println(ODR, HEX);
	//uint8_t ACT_INACT_CTL_Reg = SPIreadOneRegister(0x27);      Serial.print("ACT_INACT_CTL_Reg = "); Serial.println(ACT_INACT_CTL_Reg, HEX);
  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(0x0B);  // read instruction
  SPI.transfer(0x20);  // Start burst read at Reg 20
  Serial.println("Start Burst Read of all Control Regs - Library version 6-24-2012:");
  Serial.print("Reg XL362_THRESH_ACT_L   = B"); 	Serial.println(SPI.transfer(0x00), BIN);
  Serial.print("Reg XL362_THRESH_ACT_H   = B"); 	Serial.println(SPI.transfer(0x00), BIN);
  Serial.print("Reg XL362_TIME_ACT       = B"); 	Serial.println(SPI.transfer(0x00), BIN);
  Serial.print("Reg XL362_THRESH_INACT_L = B"); 	Serial.println(SPI.transfer(0x00), BIN);
  Serial.print("Reg XL362_THRESH_INACT_H = B"); 	Serial.println(SPI.transfer(0x00), BIN);
  Serial.print("Reg XL362_TIME_INACT_L   = B"); 	Serial.println(SPI.transfer(0x00), BIN);
  Serial.print("Reg XL362_TIME_INACT_H   = B"); 	Serial.println(SPI.transfer(0x00), BIN);
  Serial.print("Reg XL362_ACT_INACT_CTL  = B"); 	Serial.println(SPI.transfer(0x00), BIN);
  Serial.print("Reg XL362_FIFO_CONTROL   = B"); 	Serial.println(SPI.transfer(0x00), BIN);
  Serial.print("Reg XL362_FIFO_SAMPLES   = B"); 	Serial.println(SPI.transfer(0x00), BIN);
  Serial.print("Reg XL362_INTMAP1        = B"); 	Serial.println(SPI.transfer(0x00), BIN);
  Serial.print("Reg XL362_INTMAP2        = B"); 	Serial.println(SPI.transfer(0x00), BIN);
  Serial.print("Reg XL362_FILTER_CTL     = B"); 	Serial.println(SPI.transfer(0x00), BIN);
  Serial.print("Reg XL362_POWER_CTL      = B"); 	Serial.println(SPI.transfer(0x00), BIN);
  Serial.print("Reg XL362_SELF_TEST      = B"); 	Serial.println(SPI.transfer(0x00), BIN);
  
  digitalWrite(slaveSelectPin, HIGH);
}

void ADXL362::setRange(uint8_t Range){
	// Modify range (+-2g +-4g +-8g - ADXL362 Datasheep Page 33
	// Choose RangeFlag between XL362_FILTER_FLAG_2G (default), XL362_FILTER_FLAG_4G, XL362_FILTER_FLAG_8G
  uint8_t temp = SPIreadOneRegister(XL362_FILTER_CTL);	// read Reg XL362_FILTER_CTL before modifying
#ifdef ADXL362_DEBUG
  Serial.print(  "Setting Measurement Range - Reg XL362_FILTER_CTL before = "); 
  Serial.print(temp);
#endif

	switch ( Range ) { // Range affects converting LSB to mg
	case XL362_FILTER_FLAG_2G:
	  mgperLSB = 1;
	  break;
	case XL362_FILTER_FLAG_4G:
	  mgperLSB = 2;
	  break;
	case XL362_FILTER_FLAG_8G:
	  mgperLSB = 4;
	  break;
	default:
	  // YOU SHOULDN'T BE HERE !
	  mgperLSB = 1;
	  break;
	}

  temp = temp & B00111111 | Range;
  SPIwriteOneRegister(XL362_FILTER_CTL, temp); // Write to XL362_FILTER_CTL
  delay(10);	
  
#ifdef ADXL362_DEBUG
  temp = SPIreadOneRegister(XL362_FILTER_CTL);
  Serial.print(  ", Reg after = "); 
  Serial.println(temp); 
#endif
}

void ADXL362::setBandwidth(uint8_t BandWidth){
	// modify Bandwidth - ADXL362 Datasheep Page 33
	// Choose Bandwidth between XL362_FILTER_FLAG_HBW (default), XL362_FILTER_FLAG_FBW
  uint8_t temp = SPIreadOneRegister(XL362_FILTER_CTL);	// read Reg XL362_FILTER_CTL before modifying
#ifdef ADXL362_DEBUG
  Serial.print(  "Setting BandWidth - Reg XL362_FILTER_CTL before = "); 
  Serial.print(temp);
#endif

  temp = temp & B11101111 | BandWidth;
  SPIwriteOneRegister(XL362_FILTER_CTL, temp); // Write to XL362_FILTER_CTL
  delay(10);	
  
#ifdef ADXL362_DEBUG
  temp = SPIreadOneRegister(XL362_FILTER_CTL);
  Serial.print(  ", Reg after = "); 
  Serial.println(temp); 
#endif
}

void ADXL362::setOutputDatarate(uint8_t ODR){
	// modify Output Data Rate - ADXL362 Datasheep Page 33
	// Choose ODR between  XL362_FILTER_FLAG_ODR12, XL362_FILTER_FLAG_ODR25, XL362_FILTER_FLAG_ODR50, XL362_FILTER_FLAG_ODR100 (default), XL362_FILTER_FLAG_ODR200 , XL362_FILTER_FLAG_ODR400
  uint8_t temp = SPIreadOneRegister(XL362_FILTER_CTL);	// read Reg XL362_FILTER_CTL before modifying
#ifdef ADXL362_DEBUG
  Serial.print(  "Setting Output Data Rate - Reg XL362_FILTER_CTL before = "); 
  Serial.print(temp);
#endif

  temp = temp & B11111000 | ODR;
  SPIwriteOneRegister(XL362_FILTER_CTL, temp); // Write to XL362_FILTER_CTL
  delay(10);	
  
#ifdef ADXL362_DEBUG
  temp = SPIreadOneRegister(XL362_FILTER_CTL);
  Serial.print(  ", Reg after = "); 
  Serial.println(temp); 
#endif
}

void ADXL362::setNoiseLevel(uint8_t NoiseLevel){
	// modify Noise Level - ADXL362 Datasheep Page 34
	// Choose NoiseLevel between XL362_POWER_FLAG_NOISE_NORMAL (default), XL362_POWER_FLAG_NOISE_LOW, XL362_POWER_FLAG_NOISE_ULTRALOW
  uint8_t temp = SPIreadOneRegister(XL362_POWER_CTL);	// read Reg XL362_FILTER_CTL before modifying
#ifdef ADXL362_DEBUG
  Serial.print(  "Setting Output Data Rate - Reg XL362_POWER_CTL before = "); 
  Serial.print(temp);
#endif

  temp = temp & B11001111  | NoiseLevel;
  SPIwriteOneRegister(XL362_POWER_CTL, temp); // Write to XL362_FILTER_CTL
  delay(10);	
  
#ifdef ADXL362_DEBUG
  temp = SPIreadOneRegister(XL362_POWER_CTL);
  Serial.print(  ", Reg after = "); 
  Serial.println(temp); 
#endif
}

// Basic SPI routines to simplify code
// read and write one register

uint8_t ADXL362::SPIreadOneRegister(uint8_t regAddress){
  uint8_t regValue = 0;
  
  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(0x0B);  // read instruction
  SPI.transfer(regAddress);
  regValue = SPI.transfer(0x00);
  digitalWrite(slaveSelectPin, HIGH);

  return regValue;
}

void ADXL362::SPIwriteOneRegister(uint8_t regAddress, uint8_t regValue){
  
  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(0x0A);  // write instruction
  SPI.transfer(regAddress);
  SPI.transfer(regValue);
  digitalWrite(slaveSelectPin, HIGH);
}

int ADXL362::SPIreadTwoRegisters(uint8_t regAddress){
  int twoRegValue = 0;
  
  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(0x0B);  // read instruction
  SPI.transfer(regAddress);  
  twoRegValue = SPI.transfer(0x00);
  twoRegValue = twoRegValue + (SPI.transfer(0x00) << 8);
  digitalWrite(slaveSelectPin, HIGH);

  return twoRegValue;
}  

void ADXL362::SPIwriteTwoRegisters(uint8_t regAddress, int twoRegValue){
  
  uint8_t twoRegValueH = twoRegValue >> 8;
  uint8_t twoRegValueL = twoRegValue;
  
  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(0x0A);  // write instruction
  SPI.transfer(regAddress);  
  SPI.transfer(twoRegValueL);
  SPI.transfer(twoRegValueH);
  digitalWrite(slaveSelectPin, HIGH);
}
