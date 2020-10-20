// RFM69 library: LowPowerLab (adapted)
// **********************************************************************************
// Creative Commons Attrib Share-Alike License
// You are free to use/extend this library but please abide with the CCSA license:
// http://creativecommons.org/licenses/by-sa/3.0/
// 2013-06-14 (C) felix@lowpowerlab.com
// **********************************************************************************

#include <Arduino.h>
#include <SPI.h>
#include "RFM69_registers.h"  

// RFM69 library
#define RF69_MODE_SLEEP       0         // XTAL OFF
#define RF69_MODE_STANDBY     1         // XTAL ON
#define RF69_MODE_SYNTH	      2         // PLL ON
#define RF69_MODE_RX          3         // RX MODE
#define RF69_MODE_TX	        4         // TX MODE
#define RF69_MODE_LISTEN      5         // LISTEN ON 

// RFM69 library
extern volatile byte _mode;
extern byte _powerLevel;
extern bool _isRFM69HW;
extern byte _slaveSelectPin;

byte RFM69readReg(byte addr);
void RFM69writeReg(byte addr, byte value);
void RFM69setHighPower(bool onOff);
void RFM69setHighPowerRegs(bool onOff);
void RFM69setMode(byte newMode);
void RFM69readAllRegs();
void RFM69rcCalibration();

// ********************************************************************************************************

void RFM69init(
  byte rmx_config[][2]      // RM1 / RM2 register configuration  (see radio_lib.h)
) {

  for (byte i = 0; rmx_config[i][0] != 255; i++)
    RFM69writeReg(rmx_config[i][0], rmx_config[i][1]);

//  RFM69setHighPower(true); //called regardless if it's a RFM69W or RFM69HW         <==== PROBLEM !!!!!!!!
  
  RFM69setMode(RF69_MODE_STANDBY);
  while ((RFM69readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00);            // Wait for ModeReady
}

byte RFM69readReg(byte addr)
{
  noInterrupts();
  digitalWrite(_slaveSelectPin, LOW);
  SPI.transfer(addr & 0x7F);
  byte regval = SPI.transfer(0);
  digitalWrite(_slaveSelectPin, HIGH);
  interrupts();
  return regval;
}

void RFM69writeReg(byte addr, byte value)
{
  noInterrupts();
  digitalWrite(_slaveSelectPin, LOW);
  SPI.transfer(addr | 0x80);
  SPI.transfer(value);
  digitalWrite(_slaveSelectPin, HIGH);
  interrupts();
}

void RFM69setHighPower(bool onOff) {
  _isRFM69HW = onOff;
  RFM69writeReg(REG_OCP, _isRFM69HW ? RF_OCP_OFF : RF_OCP_ON);
  if (_isRFM69HW) //turning ON
    RFM69writeReg(REG_PALEVEL, (RFM69readReg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); //enable P1 & P2 amplifier stages
  else
    RFM69writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | _powerLevel);  //enable P0 only
}

void RFM69setHighPowerRegs(bool onOff) {
  RFM69writeReg(REG_TESTPA1, onOff ? 0x5D : 0x55);
  RFM69writeReg(REG_TESTPA2, onOff ? 0x7C : 0x70);
}

void RFM69setMode(byte newMode)
{
	switch (newMode) {
		case RF69_MODE_TX:
			RFM69writeReg(REG_OPMODE, (RFM69readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
                        if (_isRFM69HW) RFM69setHighPowerRegs(true);
			break;
		case RF69_MODE_RX:
			RFM69writeReg(REG_OPMODE, (RFM69readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
                        if (_isRFM69HW) RFM69setHighPowerRegs(false);
			break;
		case RF69_MODE_SYNTH:
			RFM69writeReg(REG_OPMODE, (RFM69readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
			break;
		case RF69_MODE_STANDBY:
			RFM69writeReg(REG_OPMODE, (RFM69readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
			break;
		case RF69_MODE_SLEEP:
			RFM69writeReg(REG_OPMODE, (RFM69readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
			break;
		case RF69_MODE_LISTEN:
			RFM69writeReg(REG_OPMODE, (RFM69readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_LISTEN_ON);
			break;
		default: return;
	}

	while (_mode == RF69_MODE_SLEEP && (RFM69readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady

	_mode = newMode;
}

void RFM69readAllRegs()
{
  byte regVal;	
  for (byte regAddr = 1; regAddr <= 0x4F; regAddr++) {
    noInterrupts();
    digitalWrite(_slaveSelectPin, LOW);
    SPI.transfer(regAddr & 0x7f);
    regVal = SPI.transfer(0);
    digitalWrite(_slaveSelectPin, HIGH);
    interrupts();
    Serial.print(regAddr, HEX);
    Serial.print(" - ");
    Serial.print(regVal,HEX);
    Serial.print(" - ");
    Serial.println(regVal,BIN);
  }
  digitalWrite(_slaveSelectPin, HIGH);
  interrupts();
}

void RFM69rcCalibration()
{
  RFM69writeReg(REG_OSC1, RF_OSC1_RCCAL_START);
  while ((RFM69readReg(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);
}
