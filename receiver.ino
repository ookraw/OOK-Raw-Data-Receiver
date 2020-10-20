// !!! Arduino 1.8.8                       !!!
// !!! Board: "Arduino Pro or Pro Mini"    !!!
// !!! Processor: "ATmega328P (5V, 16MHz)" !!!
// !!! SERIAL_BAUD  9600                   !!!
// !!! Radios: see "radio_lib.cpp"         !!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

/*
  reception parameters (rp)
  --------------------
  output_option           1: output with trace; 0: output without trace 
  rp.radio_module         1: RM1 (433MHz);      2: RM2 (868MHz)
  rp.radio_frequency      in function of the selected radio module
  rp.radio_sensitivity    threshold >=  ~18dBm
  rp.max_length           cutoff length < buffer size
  rp_min_length           overruling length <= cutoff length
  
  When asked to enter the reception parameters, you may 
  copy/paste your own or one of the following parameter lines:
  1 1 433.864 20 200 32
  1 1 433.864 18 600 32
  1 1 433.920 30  32 32
  1 2 868.210 40  32 32
  1 2 868.970 17  32 32
*/ 
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
 
  *************************
  * OOK RAW DATA RECEIVER *
  *************************

  Self-Contained Workspace
  ------------------------
  This Arduino workspace contains all that is needed: 
  - receiver.ino    (this file)
  - categorizer.cpp
  - categorizer.h
  - categorizer_lib.cpp
  - recorder.cpp
  - radio_lib.cpp
  - radio_lib.h
  - RFM69_lib.cpp
  - RFM69_registers.h

  Off-Line Processing
  -------------------
  The following defines the interface to the TRACE if you want to run the categorizer off-line:
  - !TRACE!: start marker
  - rs.duration[1] contains the first HIGH duration (at index position 1 not zero!)
  - rs.count is without end-record, it is the index of the last LOW
  - end-record is either a pause (x, CEIL) or a zero duration (0, 0) 
    x is the terminating HIGH duration AFTER the last LOW
  - the following records will be appended after the end of the TRACE (see reporting):  
    - checkout record   : (checksum, rs.unreliable_count)
    - end-of-data record: (-1, -1)
    
  rs: recorded_signals (cf. radio_lib.h)
*/
// ********************************************************************************************************

#include <Arduino.h>
#include <SPI.h>
#include "radio_lib.h"  
#include "categorizer.h"  

// interaction
#define LED            13
#define SERIAL_BAUD  9600

// default values
#define TRACE_OUTPUT               1     // 1: output with trace; 0: output without trace   
#define RADIO_MODULE_1          RM_1     
#define RADIO_FREQUENCY_1    433.864      
#define RADIO_MODULE_2          RM_2     
#define RADIO_FREQUENCY_2    868.240      
#define RADIO_SENSITIVITY         18     // minimal REG_OOKFIX value to trigger the start of a reception   
#define RECEPTION_MAX_LENGTH     150     // maximal length of a received signal sequence (cutoff length)
#define RECEPTION_MIN_LENGTH      32     // minimal length overruling return codes (overrules an abort)

#define FRQ(x) ((long) ( x * (1<<14) ))  // floating to long conversion used for frequencies

byte output_option;       // 0: blinks only, 1: long printout, 2: short printout
int  rp_min_length;       // minimal length in case of abort
receiver_parameters rp;   // receiver parameters

// RFM69 library
volatile byte _mode;
byte _powerLevel;
bool _isRFM69HW;
byte _slaveSelectPin;

void reporting(recorded_signals &rs);
void blink_led(byte pin, int delay_high, int delay_low, int rep);

// HIGH/LOW duration-categories:   [odd indices]: HIGH-durations, [even indices]: LOW-durations
// ----------------------------
categories  duration_category[2]; 

// accumulated noise between two receptions (recorder return codes)
//------------------
byte acc_err[NR];
byte acc_ind;

// buffers
// -------
uint8_t  uint8buf32[DIM_32];    // uint8_t  buffer
uint16_t uint16buf64[DIM_64];   // uint16_t buffer

// ========================================================================================================

void setup() {
  pinMode(LED, OUTPUT); 
  Serial.begin(SERIAL_BAUD);
  
  // start
  // -----
  Serial.println(F("receiver 20.10.2020"));  

  // reception parameter default values
  // ----------------------------------
  output_option=         TRACE_OUTPUT; 
  rp.radio_module=       RADIO_MODULE_1; 
  rp.radio_frequency=    FRQ(RADIO_FREQUENCY_1);
  rp.radio_sensitivity=  RADIO_SENSITIVITY;
  rp.max_length=     min(RECEPTION_MAX_LENGTH, NV);         
  rp_min_length=     min(RECEPTION_MIN_LENGTH, rp.max_length);
  
  // get reception parameters
  // ------------------------
  Serial.println(F("paste/enter parameters within 3 seconds:"));
  delay(3000);    
  if (Serial.available() > 1) output_option= Serial.parseInt();
  if (Serial.available() > 1) rp.radio_module= Serial.parseInt();
  if (Serial.available() > 1) rp.radio_frequency= FRQ(Serial.parseFloat());
  if (Serial.available() > 1) rp.radio_sensitivity= Serial.parseInt();
  if (Serial.available() > 1) {
    rp.max_length= Serial.parseInt(); 
    rp.max_length= min(rp.max_length, NV); 
  }
  if (Serial.available() > 1) {
    rp_min_length= Serial.parseInt();
  }
  rp_min_length= min(rp_min_length, rp.max_length);
  
  // print reception parameters
  Serial.print(F("output option    :\t"));
  Serial.println(output_option);
  Serial.print(F("radio module     :\t"));
  Serial.println(rp.radio_module);  
  Serial.print(F("radio frequency  :\t"));
  Serial.println( 10*((100*rp.radio_frequency)>>4)>>10 ); 
  Serial.print(F("radio sensitivity:\t"));
  Serial.println(rp.radio_sensitivity);
  Serial.print(F("reception max. length:\t"));
  Serial.println(rp.max_length);
  Serial.print(F("reception min. length:\t"));
  Serial.println(rp_min_length);
  Serial.println();

  // initiate both radio modules to standby (cf. radio_lib.cpp)
  // --------------------------------------
  init_radio();

  // reset accumulated recorder return codes
  // ---------------------------------------
  for (acc_ind= 0; acc_ind < NR; acc_ind++) acc_err[acc_ind]= 0;

  // ready-signal: 3 blinks 
  // ======================
  blink_led(LED, 300, 300, 3);
  
}

// ========================================================================================================

void loop() {
  byte return_code;  

  // recorded signals (cf. radio_lib.cpp)
  // ================
  recorded_signals rs;
  
  // allocate buffers (first index = 1 = index of the first HIGH, position 0 is not used)
  // ----------------
  byte signal_strength[WARM_UP + 1]; // signal strengths of the first WARM_UP signals
                                     // signal strengths: [odd indices]: HIGH-strengths, [even indices]: LOW-strengths 
  uint16_t signal_duration[NV + 5];  // signal sequence : [odd indices]: HIGH-durations, [even indices]: LOW-durations
                                     // LSB flagged: 0: reliable, 1: unreliable value
                                     // 2 records appended at the end plus 1 (unused position 0) gives 5
 
  // allocate signals 
  // ----------------
  rs.duration= signal_duration;
  rs.strength= signal_strength;

  while (true) {
    
    // ======== //
    // recorder //   record HIGH- / LOW- signal durations
    // ======== //
    // return_code: see radio_lib.h
    return_code= recorder(rp, rs);    
    
    // accumulated recorder return codes (-> noise)
    // ---------------------------------
    if ((return_code < NR) && (acc_err[return_code] < 100)) acc_err[return_code]++;

    // check the start signal 
    if (return_code == RRC_6) {
      
      // the first HIGH is too strong: 1 long blink 
      // ----------------------------
      Serial.println();
      Serial.print(F("***** signal too strong: "));
      Serial.print(rs.ref_strength_high);
      Serial.println(F(" dBm"));
      blink_led (LED, 600, 600, 1);
      continue;
    }
    if ((return_code > 1) && (rs.count < rp_min_length)) {
      // recorder ended with error
      // -------------------------
      continue;
    }
    if (output_option == TRACE_OUTPUT) {
      // *********** //
      // print trace //
      // *********** //
      reporting(rs);
    }

    // accumulated noise since previous reception
    // ------------------------------------------
    Serial.println();
    Serial.println(F("accumulated recorder return codes:"));
    for (acc_ind= 0; acc_ind < NR; acc_ind++) {
      if (acc_err[acc_ind] > 0) {
        Serial.print(acc_ind);
        Serial.print("\t");
        Serial.println(acc_err[acc_ind]);
      }
    }

    // reception summery
    // -----------------
    Serial.print(F("recorder return_code: "));   
    Serial.print(return_code);   
    Serial.print(F(", count: "));   
    Serial.print(rs.count);   
    Serial.print(F(", unreliables: "));   
    Serial.println(rs.unreliable_count);
    Serial.println();
    
    // =========== //
    // categorizer //   map the signal durations into duration levels
    // =========== //      
    // return_code: see categorizer.h
    return_code= 0;
    categorizer (duration_category, signal_duration, rs.count, rs.unreliable_count, return_code,
                 uint8buf32, uint16buf64);
    Serial.print(F("categorizer return_code: "));   
    Serial.println(return_code);   
    
    // reset the accumulated recorder return codes
    for (acc_ind= 0; acc_ind < NR; acc_ind++) acc_err[acc_ind]= 0;
    
    // successful reception: 1 short blink
    // ====================
    if (return_code == CRC_0) blink_led (LED, 300, 300, 1);
  }
} // end void loop()

// ========================================================================================================
//*********************************************************************************************************

void reporting(recorded_signals &rs) {
  // *********** //
  // print trace //
  // *********** //
  
  int  ind;  
  int  k;
  // Fletcher16 Checksum (cf. categorizer_lib.cpp)
  unsigned int sum1;  
  unsigned int sum2; 
  
  k= min(WARM_UP + 1, rs.count); 
  if (k > 0) {
    
    // print signal strengths obtained during warm-up
    // ----------------------------------------------
    Serial.println();
    Serial.println(F("signal-strength"));
    // index of first HIGH
    ind= 1;
    while (ind < k) {
      Serial.print(ind);
      // HIGH
      Serial.print("\t");
      Serial.print(rs.strength[ind]);
      if (++ind >= k) break;
      Serial.print(" / ");
      // LOW
      Serial.print(rs.strength[ind]);
      if (++ind >= k) break;
      Serial.println();
    }
    
    // print TRACE: all duration values, "ending" included
    // ---------------------------------------------------
    // (ending is either a pause (x, CEIL) or a zero duration (0, 0))
    Serial.println();
    Serial.println(F("!TRACE!"));
    // index of first HIGH
    ind= 1;
    // index of "ending" (0 or CEIL)
    k= rs.count + 2;
    // initiate checksum
    sum1= 0;
    sum2= 0;
    while (ind <= k) {
      Serial.print(ind);
      // HIGH
      Serial.print("\t");
      Serial.print(rs.duration[ind]);
      sum1= (sum1 + rs.duration[ind]) % 255;
      sum2= (sum2 + sum1) % 255;
      if (++ind > k) break;
      // LOW
      Serial.print("\t");
      Serial.print(rs.duration[ind]);    
      sum1= (sum1 + rs.duration[ind]) % 255;
      sum2= (sum2 + sum1) % 255;
      if (++ind > k) break;
      Serial.println();
    }
    
    // print checkout record   (checksum, rs.unreliable_count)
    // ---------------------
    Serial.println();
    Serial.print((sum2 << 8) | sum1);
    Serial.print("\t");
    Serial.println(rs.unreliable_count);    
    
    // print end-of-data record (-1, -1)
    // ------------------------
    Serial.print("-1");
    Serial.print("\t -1");
    Serial.println();

  }

  // print reference strength
  // ------------------------
  Serial.println();
  Serial.print(F("ref_strength: "));
  Serial.print(rs.ref_strength_high);
  Serial.print(" ");
  Serial.print(rs.ref_strength_low);
  Serial.println();
}

//*********************************************************************************************************

void blink_led(byte pin, int delay_high, int delay_low, int rep)
{
  pinMode(pin, OUTPUT);
  // blink
  for (int i=0; i<rep; i++) {
    digitalWrite(pin, HIGH);
    delay(delay_high);
    digitalWrite(pin, LOW);
    delay(delay_low);
  }
}

//*********************************************************************************************************
