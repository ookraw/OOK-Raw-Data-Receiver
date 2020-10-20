// !!! Radio: Hope rfm69w    !!!
// !!! MCU  : mini pro 3.3V  !!!
// !!!        MEGA328P       !!!
// !!! Radio-MCU Connections !!!
// !!!         RM1 RM2       !!!
// !!!   DIO0   3   2        !!!
// !!!      1   9   7        !!!
// !!!      2   8   6        !!!
// !!!   NSS   10   5        !!!
// !!!   MOSI  11  11        !!!
// !!!   MISO  12  12        !!!
// !!!   SCK   13  13        !!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!

/*
  Copyright Felix Baessler, felix.baessler@gmail.com
  This software is released under CC-BY-NC 4.0.
  The licensing TLDR; is: You are free to use, copy, distribute and transmit this Software for personal, 
  non-commercial purposes, as long as you give attribution and share any modifications under the same license. 
  Commercial or for-profit use requires a license. 
  SEE FULL LICENSE DETAILS HERE: https://creativecommons.org/licenses/by-nc/4.0/

  OOK Raw Data Receiver
  0. Radio Library
  1. Recorder
  2. Categorizer
  3. Categorizer Library

  ====================
  = 0. Radio Library =  remove glitches/bounces from the current signal and get the strength of the next signal 
  ====================

  0. Radio Library
  0.1 Radio Module 1
  0.1.1 RM1 Loop While High
  0.1.1 RM1 Loop While Low
  0.2 Radio Module 2
  0.2.1 RM2 Loop While High
  0.2.2 RM2 Loop While Low  
  0.3 Poll Selected Radio (Wrapper)
  0.3.1 Loop While High
  0.3.2 Loop While Low
  0.4 Helper
  0.4.1 Radio Initialization 
  0.4.2 SPI Begin
  0.4.3 Get Signal Strength
  0.4.4 Set Mode
  0.4.5 Set Frequency
  0.4.6 Set Threshold
  0.4.7 Set Power
 */
 
#include <Arduino.h>
#include <SPI.h>
#include "RFM69_registers.h"  
#include "radio_lib.h" 

// durations (number of polling cycles)
// ---------
#define SPIKE_HIGH            8         // high trigger  > 8: the spike is a HIGH 
#define DROP_LOW              8         // low  trigger  > 8: the drop  is a LOW
#define TRIGGER_LOW          16         // min LOW  to trigger the end of a HIGH (shortest acceptable low  duration)
#define TRIGGER_HIGH         48         // min HIGH to trigger the end of a LOW  (shortest acceptable high duration)
#define CEIL_UI           65000U        // limit of recorded HIGH durations / limit of Unsigned Integers (16 bits)
#define CEIL_UI_X2       130000UL       // limit of recorded LOW  durations (2 * CEIL_UI)
#define CEIL_UL      4294967000UL       // limit of measured LOW  durations (UL: Unsigned Long ~ 32 bits)  
#define LC2                   2         // lost cycles
#define LC25                 25         // lost cycles
#define LC100               100         // lost cycles
#define RFM69_0X00 0

// RM 1: radio module 1 connection
// -------------------------------
// slave select SS1
// dclk <-- pin 9
#define RFM69_1_DIO1_DDR  DDRB
#define RFM69_1_DIO1_PIN  PINB
#define RFM69_1_DIO1_PORT PORTB         // B (digital pins 8 to 13) 
#define RFM69_1_DIO1_MASK (1 << 1)      // DIO1 <-- pin 9  (RM1 dclk set)
// data <-> pin 8
#define RFM69_1_DIO2_DDR  DDRB
#define RFM69_1_DIO2_PIN  PINB
#define RFM69_1_DIO2_PORT PORTB        
#define RFM69_1_DIO2_MASK (1 << 0)      // DIO2 <-> pin 8  (RM1 data send/receive)
#define DATA_1_PIN        8

// RM 2: radio module 2 connection
// -------------------------------
// slave select SS2
// dclk <-- pin 7
#define RFM69_2_DIO1_DDR DDRD
#define RFM69_2_DIO1_PIN PIND
#define RFM69_2_DIO1_PORT PORTD         // D (digital pins 0 to 7) 
#define RFM69_2_DIO1_MASK (1 << 7)      // DIO1 <-- pin 7  (RM2 dclk set)
// data <-> pin 6
#define RFM69_2_DIO2_DDR DDRD
#define RFM69_2_DIO2_PIN PIND
#define RFM69_2_DIO2_PORT PORTD        
#define RFM69_2_DIO2_MASK (1 << 6)      // DIO2 <-> pin 6  (RM2 data send/receive)
#define DATA_2_PIN        6

// RFM69 library
// -------------
extern volatile byte  _mode;
extern byte     _powerLevel;
extern bool      _isRFM69HW;
extern byte _slaveSelectPin;
void RFM69init(byte rmx_config[][2]);
void RFM69setMode(byte newMode);
void RFM69rcCalibration();
void RFM69writeReg(byte addr, byte value);
byte RFM69readReg(byte addr);

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// ****************** //
// 0.1 Radio Module 1 //  
// ****************** //
// 0.1.1 RM1 Loop While High
// 0.1.1 RM1 Loop While Low
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//******************************* begin rm1_loop_while_high ************************************************************************
byte rm1_loop_while_high(unsigned int &duration_high, unsigned long &duration_low, byte &strength_low) 
{
  // ------------------------- //
  // 0.1.1 RM1 LOOP WHILE HIGH //
  // ------------------------- //
  // loop as long as the signal is high 
  // at beginning:   duration_high >= TRIGGER_HIGH (passed from previous rm1_loop_while_low)
  // at termination: duration_low  >  TRIGGER_LOW  ("sufficiently" long to start a new LOW) 
  //                 duration_high :  total HIGH duration 
  //                 strength_low  :  signal strength of the following LOW
  // return codes:
  // RRC_1 : end of HIGH
  // RRC_3 : overflow on HIGH (duration_high >= CEIL_UI)
  // RRC_4 : excessive bouncing on HIGH
  // NB: a duration_high in front of a reception end is often exceptionally short 
  //     a duration_high >= CEIL_UI is considered as an anomaly -> return RRC_3 
  
      unsigned int  accumulated_duration;   // accumulated duration while bouncing
      unsigned int  temp_duration;          // temp duration 
  
      duration_low= 0;
      strength_low= 0;           
      // begin HIGH-loop
      while(true) {   
        // continue after a drop ( <= TRIGGER_LOW )
        //      AND a long HIGH  ( > SPIKE_HIGH )
        
        temp_duration= LC2; 
        while((RFM69_1_DIO2_PIN & RFM69_1_DIO2_MASK) == RFM69_1_DIO2_MASK) {
          // counterbalance bouncing loop, which is slower 
          __asm__ __volatile__ ("nop\n");
          // overflow on HIGH
          if (++temp_duration >= CEIL_UI) return RRC_3;                           // high overflow         -------> return RRC_3
        }   
        duration_high+= temp_duration;
        // potential low detected
        
        // bouncing loop : loop as long as the drop duration <= TRIGGER_LOW
        // expect dozens of bounces at high sensitivity
        // assumption: spike and drop cycles are of about equal duration      
        accumulated_duration= 0;
        do { 
          // is this a genuine LOW or just a drop?
          temp_duration= LC2;
          while((RFM69_1_DIO2_PIN & RFM69_1_DIO2_MASK) == RFM69_0X00) {
            if (++temp_duration > TRIGGER_LOW) {goto EOB;}                        // End-Of-Bouncing       -------> EOB 
          } 
          accumulated_duration+= temp_duration;         
          // a drop has been detected
          // is the following HIGH a genuine HIGH or just a spike?
          // (the sensitivity of SPIKE_HIGH is surprisingly small)
          temp_duration= LC2;
          while((RFM69_1_DIO2_PIN & RFM69_1_DIO2_MASK) == RFM69_1_DIO2_MASK) {
            __asm__ __volatile__ ("nop\n");
            if (++temp_duration > SPIKE_HIGH) {goto CWH;}                         // Continue-With-High    -------> CWH  
          } 
          accumulated_duration+= temp_duration;          
          // after a drop followed a spike
        } while (accumulated_duration < CEIL_UI);  
        // excessive bouncing on HIGH
        return RRC_4;                                                             // too much bouncing     -------> return RRC_4
      
EOB:    // End-Of-Bouncing
        // get signal_strength
        // +++++++++++++++++++
        strength_low= signal_strength(); 
        duration_low= accumulated_duration + temp_duration + LC100;
        // end of HIGH
        return RRC_1;                                                             // end of High           -------> return RRC_1 

CWH:    // continue with high     
        duration_high+= accumulated_duration + temp_duration; 
        // overflow on HIGH
        if (duration_high >= CEIL_UI) return RRC_3;                               // high overflow         -------> return RRC_3
      }  
      // end of loop on HIGH  (while(true))                                       // continue high loop
}
//******************************* end rm1_loop_while_high **************************************************************************

//******************************* begin rm1_loop_while_low *************************************************************************
byte rm1_loop_while_low(unsigned int &duration_high, unsigned long &duration_low, byte &strength_high, unsigned long duration_low_limit) 
{
  // ------------------------ //
  // 0.1.2 RM1 LOOP WHILE LOW //
  // ------------------------ //
  // loop as long as the signal is low 
  // at beginning:   duration_low  >= TRIGGER_LOW  (passed from previous rm1_loop_while_high)
  //                 duration_low_limit : LOW "timeout" duration -> end of reception
  // at termination: duration_high >  TRIGGER_HIGH ("sufficiently" long to start a new HIGH) 
  //                 duration_low   : total LOW duration
  //                 strength_high  : signal strength of following HIGH
  // return code:
  // RRC_0 : end of Reception: (duration_low >= duration_low_limit, cf. recorder: INFINITE_PAUSE or LONG_PAUSE)
  // RRC_1 : end of LOW
  // RRC_2 : excessive bouncing on LOW

     unsigned int accumulated_duration;      // accumulated duration while bouncing
     unsigned int temp_duration;             // temp duration 
  
      duration_high= 0;
      strength_high= 0;           
      // begin LOW-loop
      while(true) {       
        // continue after a spike  ( <= TRIGGER_HIGH ) 
        //          AND a long LOW ( >  DROP_LOW ) 
        
        temp_duration= LC2; 
        while((RFM69_1_DIO2_PIN & RFM69_1_DIO2_MASK) == RFM69_0X00) {
          // counterbalance bouncing loop, sightly slower 
          __asm__ __volatile__ ("nop\n");
          if (++temp_duration >= CEIL_UI) {
            duration_low+= temp_duration;
            if (duration_low >= duration_low_limit) {
              duration_low= min(duration_low_limit, CEIL_UI_X2);
              strength_high= 0;
              // end of Reception           
              return RRC_0;                                                      // End-Of-Reception      -------> return RRC_0
            }
            temp_duration= LC2; 
          }  
        }        
        duration_low+= temp_duration;
        // potential high detected
        
        // bouncing loop : loop as long as the spike duration <= TRIGGER_HIGH
        // expect much more bounces than during the HIGH-loop
        // assumption: spike and drop cycles are of about equal duration      
        accumulated_duration= 0;
        do {  
          // is this a genuine HIGH or just a spike?
          temp_duration= LC2;
          while((RFM69_1_DIO2_PIN & RFM69_1_DIO2_MASK) == RFM69_1_DIO2_MASK) {
            __asm__ __volatile__ ("nop\n");
            __asm__ __volatile__ ("nop\n");
            if (++temp_duration > TRIGGER_HIGH) {goto EOB;}                      // End-Of-Bouncing       -------> EOB
          } 
          accumulated_duration+= temp_duration; 
          // a spike has been detected
          // is the following LOW a genuine LOW or just a drop?
          temp_duration= LC2;
          while((RFM69_1_DIO2_PIN & RFM69_1_DIO2_MASK) == RFM69_0X00) {
            __asm__ __volatile__ ("nop\n");
            if (++temp_duration > DROP_LOW) {goto CWL;}                          // Continue-With-Low     -------> CWL 
          }
          accumulated_duration+= temp_duration; 
          // after a spike followed a drop
        } while (accumulated_duration <= CEIL_UI);                               // continue bouncing loop  
        // excessive bouncing on LOW
        return RRC_2;                                                            // too much bouncing     -------> return RRC_2
        
EOB:    // End-Of-Bouncing
        // get signal_strength
        // +++++++++++++++++++
        strength_high= signal_strength();       
        duration_high= accumulated_duration + temp_duration + LC100; 
        // end of LOW
        return RRC_1;                                                            // end of LOW            -------> return RRC_1 

CWL:    // Continue-With-Low
        duration_low+= accumulated_duration + temp_duration; 
        if (duration_low >= duration_low_limit) {
          duration_low= min(duration_low_limit, CEIL_UI_X2);
          strength_high= 0;           
          // end of Reception           
          return RRC_0;                                                          // End-Of-Reception      -------> return RRC_0
        }
      }                                                                          // continue low loop
      // end of loop on LOW (while(true))
}
//******************************* end rm1_loop_while_low **************************************************************************

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// ****************** //
// 0.2 Radio Module 2 //
// ****************** //
// 0.2.1 RM2 Loop While High
// 0.2.1 RM2 Loop While Low
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//******************************* begin rm2_loop_while_high ************************************************************************
byte rm2_loop_while_high(unsigned int &duration_high, unsigned long &duration_low, byte &strength_low) 
{
  // ------------------------- //
  // 0.2.1 RM2 LOOP WHILE HIGH //
  // ------------------------- //
  // loop as long as the signal is high 
  // at beginning:   duration_high >= TRIGGER_HIGH (passed from previous rm1_loop_while_low)
  // at termination: duration_low  >  TRIGGER_LOW  ("sufficiently" long to start a new LOW) 
  //                 duration_high  : total HIGH duration 
  //                 strength_low   : signal strength of the following LOW
  // return codes:
  // RRC_1 : end of HIGH
  // RRC_3 : duration_high >= CEIL_UI (HIGH overflow)
  // RRC_4 : too much HIGH bouncing
  // NB: a duration_high in front of a reception end is often exceptionally short 
  //     a duration_high >= CEIL_UI is considered as an anomaly -> return RRC_3 
  
      unsigned int  accumulated_duration;      // accumulated duration while bouncing
      unsigned int  temp_duration;             // temp duration 
  
      duration_low= 0;
      strength_low= 0;           
      // begin HIGH-loop
      while(true) {   
        // continue after a drop ( <= TRIGGER_LOW )
        //      AND a long HIGH  ( > SPIKE_HIGH )
        
        temp_duration= LC2; 
        while((RFM69_2_DIO2_PIN & RFM69_2_DIO2_MASK) == RFM69_2_DIO2_MASK) {
          // counterbalance bouncing loop, which is slower 
          __asm__ __volatile__ ("nop\n");
          if (++temp_duration >= CEIL_UI) return RRC_3;                          // high overflow         -------> return RRC_3
        }   
        duration_high+= temp_duration;
        // potential low detected
        
        // bouncing loop : loop as long as the drop duration <= TRIGGER_LOW
        // expect dozens of bounces at high sensitivity
        // assumption: spike and drop cycles are of about equal duration      
        accumulated_duration= 0;
        do { 
          // is this a genuine LOW or just a drop?
          temp_duration= LC2;
          while((RFM69_2_DIO2_PIN & RFM69_2_DIO2_MASK) == RFM69_0X00) {
            if (++temp_duration > TRIGGER_LOW) {goto EOB;}                       // End-Of-Bouncing       -------> EOB 
          } 
          accumulated_duration+= temp_duration;         
          // a drop has been detected
          // is the following HIGH a genuine HIGH or just a spike?
          // (the sensitivity of SPIKE_HIGH is surprisingly small)
          temp_duration= LC2;
          while((RFM69_2_DIO2_PIN & RFM69_2_DIO2_MASK) == RFM69_2_DIO2_MASK) {
            __asm__ __volatile__ ("nop\n");
            if (++temp_duration > SPIKE_HIGH) {goto CWH;}                        // Continue-With-High    -------> CWH  
          } 
          accumulated_duration+= temp_duration;          
          // after a drop followed a spike
        } while (accumulated_duration < CEIL_UI);  
        // too much bouncing
        return RRC_4;                                                            // too much bouncing     -------> return RRC_4
      
EOB:    // End-Of-Bouncing
        // get signal_strength
        // +++++++++++++++++++
        strength_low= signal_strength(); 
        duration_low= accumulated_duration + temp_duration + LC100;
        return RRC_1;                                                            // end of High           -------> return RRC_1 

CWH:    // continue with high     
        duration_high+= accumulated_duration + temp_duration; 
        if (duration_high >= CEIL_UI) return RRC_3;                              // high overflow         -------> return RRC_3
      }  
      // end of loop on HIGH  (while(true))                                      // continue high loop
}
//******************************* end rm2_loop_while_high **************************************************************************

//******************************* begin rm2_loop_while_low *************************************************************************
byte rm2_loop_while_low (unsigned int &duration_high, unsigned long &duration_low, byte &strength_high, unsigned long duration_low_limit) 
{
  // ------------------------ //
  // 0.2.2 RM2 LOOP WHILE LOW //
  // ------------------------ //
  // loop as long as the signal is low 
  // at beginning:   duration_low  >= TRIGGER_LOW  (passed from previous rm1_loop_while_high)
  //                 duration_low_limit : LOW "timeout" duration -> end of reception
  // at termination: duration_high >  TRIGGER_HIGH ("sufficiently" long to start a new HIGH) 
  //                 duration_low   : total LOW duration
  //                 strength_high  : signal strength of following HIGH
  // return code:
  // RRC_0 : duration_low >= duration_low_limit (cf. recorder: INFINITE_PAUSE or LONG_PAUSE)
  // RRC_1 : end of LOW
  // RRC_2 : too much LOW bouncing
      
      unsigned int accumulated_duration;      // accumulated duration while bouncing
      unsigned int temp_duration;             // temp duration 
  
      duration_high= 0;
      strength_high= 0;           
      // begin LOW-loop
      while(true) {       
        // continue after a spike  ( <= TRIGGER_HIGH ) 
        //          AND a long LOW ( >  DROP_LOW ) 
        
        temp_duration= LC2; 
        while((RFM69_2_DIO2_PIN & RFM69_2_DIO2_MASK) == RFM69_0X00) {
          // counterbalance bouncing loop, sightly slower 
          __asm__ __volatile__ ("nop\n");
          if (++temp_duration >= CEIL_UI) {
            duration_low+= temp_duration;
            if (duration_low >= duration_low_limit) {
              duration_low= min(duration_low_limit, CEIL_UI_X2);
              strength_high= 0;           
              return RRC_0;                                                      // End-Of-Reception      -------> return RRC_0
            }
            temp_duration= LC2; 
          }  
        }        
        duration_low+= temp_duration;
        // potential high detected
        
        // bouncing loop : loop as long as the spike duration <= TRIGGER_HIGH
        // expect much more bounces than during the HIGH-loop
        // assumption: spike and drop cycles are of about equal duration      
        accumulated_duration= 0;
        do {  
          // is this a genuine HIGH or just a spike?
          temp_duration= LC2;
          while((RFM69_2_DIO2_PIN & RFM69_2_DIO2_MASK) == RFM69_2_DIO2_MASK) {
            __asm__ __volatile__ ("nop\n");
            __asm__ __volatile__ ("nop\n");
            if (++temp_duration > TRIGGER_HIGH) {goto EOB;}                      // End-Of-Bouncing       -------> EOB
          } 
          accumulated_duration+= temp_duration; 
          // a spike has been detected
          // is the following LOW a genuine LOW or just a drop?
          temp_duration= LC2;
          while((RFM69_2_DIO2_PIN & RFM69_2_DIO2_MASK) == RFM69_0X00) {
            __asm__ __volatile__ ("nop\n");
            if (++temp_duration > DROP_LOW) {goto CWL;}                          // Continue-With-Low     -------> CWL 
          }
          accumulated_duration+= temp_duration; 
          // after a spike followed a drop
        } while (accumulated_duration <= CEIL_UI);                               // continue bouncing loop  
        // too much bouncing
        return RRC_2;                                                            // too much bouncing     -------> return RRC_2
        
EOB:    // End-Of-Bouncing
        // get signal_strength
        // +++++++++++++++++++
        strength_high= signal_strength();       
        duration_high= accumulated_duration + temp_duration + LC100; 
        return RRC_1;                                                            // end of LOW            -------> return RRC_1 

CWL:    // Continue-With-Low
        duration_low+= accumulated_duration + temp_duration; 
        if (duration_low >= duration_low_limit) {
          duration_low= min(duration_low_limit, CEIL_UI_X2);
          strength_high= 0;           
          return RRC_0;                                                          // End-Of-Reception      -------> return RRC_0
        }
      }                                                                          // continue low loop
      // end of loop on LOW (while(true))
}
//******************************* end rm2_loop_while_low **************************************************************************

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// *********************** //
// 0.3 Poll Selected Radio //
// *********************** //
// 0.3.1 Loop While High
// 0.3.2 Loop While LOW
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//******************************* begin Polling Selected Radio ********************************************************************
byte loop_while_high  (unsigned int &duration_high, unsigned long &duration_low, byte &strength_low)
{
  // --------------------- //
  // 0.3.1 Loop While High //
  // --------------------- //
  if(_slaveSelectPin == SS1) {
    return rm1_loop_while_high(duration_high, duration_low, strength_low);
  }
  if(_slaveSelectPin == SS2) {
    return rm2_loop_while_high(duration_high, duration_low, strength_low);
  }
  return (RRC_15);
}  
//******************************* begin Polling Selected Radio ********************************************************************
byte loop_while_low   (unsigned int &duration_high, unsigned long &duration_low, byte &strength_high, unsigned long duration_low_limit)  
{
  // -------------------- //
  // 0.3.2 Loop While LOW //
  // -------------------- //
  if(_slaveSelectPin == SS1) {
    return rm1_loop_while_low(duration_high, duration_low, strength_high, duration_low_limit);  
  }
  if(_slaveSelectPin == SS2) {
    return rm2_loop_while_low(duration_high, duration_low, strength_high, duration_low_limit);  
  }
  return (RRC_15);
}  
//******************************* end Polling Selected Radio **********************************************************************

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// ********** //
// 0.4 Helper //
// ********** //
// 0.4.1 Radio Initialization 
// 0.4.2 SPI Begin
// 0.4.3 Get Signal Strength
// 0.4.4 Set Mode
// 0.4.5 Set Frequency
// 0.4.6 Set Threshold
// 0.4.7 Set Power
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//*********************************************************************************************************************************
void init_radio()
{
  // -------------------------- //
  // 0.4.1 Radio Initialization // (standby)
  // -------------------------- //
  /*
  courtesy of Frank @ https://www.sevenwatt.com/
  RSSI (in dBm) = (RawRSSI-256) / 2
  
  RFM69 OOK – DAGC, Sensitivity Boost and Modulation index
  DAGC enabled, Sensitivity Boost enabled, and as one has to choose: Modulation index: High-M.
  
  RFM69 OOK RSSI and optimal bitrate
  From the 3000bps it can be seen that the RSSI signal is sampled with about 625us intervals. 
  According to the datasheet, the RSSI sample duration depends on the duration of a bit:
      t-rssi = 2 * t-bit
  So for 3000bps, t-bit=333us, so t-rssi = 666us.
  For 32768bps, the t-rssi=61us. As the RSSI signal is recorded with 25us intervals, 
  most of the time two or three adjacent samples will have the same RSSI value.
  In conclusion, the bitrate should be high enough for reliable bit timings, 
  but not higher then needed, as a lower bitrate means a better noise filtering.
  
  RFM69 AFC failure with OOK ( automatic frequency correction)
  AFC should not be used in continuous OOK receive mode.   
  */
  
  // RM1 / RM2 register configuration
  // --------------------------------
  byte rmx_config[][2] =
  {
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_CONTINUOUSNOBSYNC | RF_DATAMODUL_MODULATIONTYPE_OOK | RF_DATAMODUL_MODULATIONSHAPING_00 }, 
    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_115200},   
    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_115200},  
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 },  // 62.5  rx bandwidth
    /* 0x1b */ { REG_OOKPEAK, RF_OOKPEAK_THRESHTYPE_FIXED}, // Selects type of threshold in the OOK data slicer: 00 → fixed
    /* 0x1d */ { REG_OOKFIX, 30},  // Fixed threshold value (in dB) in the OOK demodulator. Used when OokThresType = 00
    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 },  // Fading Margin Improvement, refer to 3.4.4;  required for RSSI !!!
    /* 0x58 */ { REG_TESTLNA, 0x2D },   // == SENSITIVITY_BOOST_HIGH
  {255, 0}
  };

  // RM1
  // --- 
  SPI_begin(RM_1);  
  RFM69init(rmx_config);       
  RFM69setMode(RF69_MODE_STANDBY);  
  RFM69rcCalibration();
  SPI.end();
  delay(100);    
  // RM2 
  // ---
  SPI_begin(RM_2);  
  RFM69init(rmx_config);     
  RFM69setMode(RF69_MODE_STANDBY); 
  RFM69rcCalibration();
  SPI.end();
  delay(100);    

  // set pin --> dclk to OUTPUT (permanent)
  RFM69_1_DIO1_DDR  |=  RFM69_1_DIO1_MASK;  // set pin --> dclk to OUTPUT
  RFM69_1_DIO1_PORT &= ~RFM69_1_DIO1_MASK;  // set pin dclk to LOW (cf. rfm69_set_data)
  RFM69_2_DIO1_DDR  |=  RFM69_2_DIO1_MASK;  // set pin --> dclk to OUTPUT
  RFM69_2_DIO1_PORT &= ~RFM69_2_DIO1_MASK;  // set pin dclk to LOW (cf. rfm69_set_data)
}
//*********************************************************************************************************************************
void SPI_begin(byte radio_module)
{
  // --------------- //
  // 0.4.2 SPI Begin //
  // --------------- //
  // setup SPI for both radio modules 
  pinMode(SS1, OUTPUT); 
  digitalWrite(SS1, HIGH);
  pinMode(SS2, OUTPUT);
  digitalWrite(SS2, HIGH);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2); // max speed, except on Due which can run at system clock speed
  SPI.begin();
  // set the _slaveSelectPin for the active radio module
  if (radio_module == RM_1) _slaveSelectPin= SS1;
  if (radio_module == RM_2) _slaveSelectPin= SS2;
  }
//*********************************************************************************************************************************
inline byte signal_strength() 
{
  // ------------------------- //
  // 0.4.3 Get Signal Strength //
  // ------------------------- //
  // Several measurements are required to obtain reproducible results. 
  // Each measurement consumes a rather large amount of time compared to short signal durations.
  // Note that RSSI measurement is also required for collision detection

  int reg_rssi_sum= 4;  // Raw RSSI (rounding of 4 measurements)  
  int strength;         // - RSSI [dBm] (dB per milliwatt)

  // avg of 4 raw rssi measurements
  for (int i=0; i<4; i++) {    
    digitalWrite(_slaveSelectPin, LOW);
    SPI.transfer(REG_RSSIVALUE & 0x7F);
    reg_rssi_sum+= SPI.transfer(0);
    digitalWrite(_slaveSelectPin, HIGH);          
  }
  // map raw rsssi -> strength (non-optimized)
  // RSSI (dBm)    = (RawRSSI – 256) / 2   =  - (128 - (RawRSSI / 2))
  strength= 128 - (reg_rssi_sum >> 3);  // half of 4 measurements
  return (strength);
}
//*********************************************************************************************************************************
void set_mode(byte mode)
{ 
  // -------------- //
  // 0.4.4 Set Mode //
  // -------------- //
  if (_slaveSelectPin == SS1) {
    // RM_1
    if (mode == RF69_MODE_RX){
      // receive on RM_1
      RFM69_1_DIO2_DDR  &= ~RFM69_1_DIO2_MASK;    // set pin data to INPUT
      RFM69_1_DIO2_PORT &= ~RFM69_1_DIO2_MASK;    // set pin data to LOW      
    } else 
    if (mode == RF69_MODE_TX){
      // send on RM_1
      RFM69_1_DIO2_DDR  |=  RFM69_1_DIO2_MASK;    // set pin data to OUTPUT
      RFM69_1_DIO2_PORT &= ~RFM69_1_DIO2_MASK;    // set pin data to LOW      
    }
  } else 
  if (_slaveSelectPin == SS2) {
    // RM_2
    if (mode == RF69_MODE_RX){
      // receive on RM_2
      RFM69_2_DIO2_DDR  &= ~RFM69_2_DIO2_MASK;    // set pin data to INPUT
      RFM69_2_DIO2_PORT &= ~RFM69_2_DIO2_MASK;    // set pin data to LOW      
    } else 
    if (mode == RF69_MODE_TX){
      // send on RM_2
      RFM69_2_DIO2_DDR  |=  RFM69_2_DIO2_MASK;    // set pin data to OUTPUT
      RFM69_2_DIO2_PORT &= ~RFM69_2_DIO2_MASK;    // set pin data to LOW      
    }
  }
  RFM69setMode(mode);  
}
//*********************************************************************************************************************************
void set_frequency(long frequency ) 
{
  // ------------------- //
  // 0.4.5 Set Frequency //
  // ------------------- //
    RFM69writeReg(REG_FRFMSB, (byte)(frequency >> 16));
    RFM69writeReg(REG_FRFMID, (byte)(frequency >>  8));
    RFM69writeReg(REG_FRFLSB, (byte)(frequency >>  0));  
}
//*********************************************************************************************************************************
void set_threshold(byte strength)
{
  // ------------------- //
  // 0.4.6 Set Threshold //
  // ------------------- //
  RFM69writeReg(REG_OOKFIX, strength);
}
//*********************************************************************************************************************************
void set_power(byte power_level)
{
  // --------------- //
  // 0.4.7 Set Power //
  // --------------- //
  RFM69writeReg(REG_PALEVEL, (RFM69readReg(REG_PALEVEL) & 0xE0) | power_level);  
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
