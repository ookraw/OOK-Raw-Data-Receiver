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

  =============================
  = Radio Library (Interface) =
  =============================
*/

// radio_modules
#define RM_1  1   // radio module 1
#define RM_2  2   // radio module 2
#define SS1  SS   // slave select RM_1
#define SS2   5   // slave select RM_2

#define RF69_MODE_SLEEP       0         // XTAL OFF
#define RF69_MODE_STANDBY     1         // XTAL ON
#define RF69_MODE_SYNTH       2         // PLL ON
#define RF69_MODE_RX          3         // RX MODE
#define RF69_MODE_TX          4         // TX MODE
#define RF69_MODE_LISTEN      5         // LISTEN ON 

// pauses (long LOW durations)
#define INFINITE_PAUSE 4294967000UL     // a "never ending" pause that preceds the start pulse   
#define LONG_PAUSE         140000UL     // minimal pause duration marking start and end of reception 
                                        // (UL: Unsigned Long ~ 32 bits)   
// for use in signal duration level
#define MSB  0b1111111111111110         // most  significant bits
#define LSB  0b0000000000000001         // least significant bit (unreliable signal flag)                              
#define DELTA_STRENGTH    5             // strength delta (dBm) between HIGH and LOW levels (reliability)
                                        // and between HIGH and reference levels (collision detection)
#define WARM_UP           8             // number of required reliable start signals   >= 8 !!!
#define NR               16             // dim recorder return codes (number of different return codes)
#define MAX_POWER        30             // maximum power of the radios

// reception return codes
// ======================
#define RRC_0   0  // End of Reception (duration_low >= LONG_PAUSE)
#define RRC_1   1  // end of HIGH / end of LOW / end of Buffer (limit= ND)
#define RRC_2   2  // excessive bouncing on LOW
#define RRC_3   3  // overflow on HIGH
#define RRC_4   4  // excessive bouncing on HIGH
#define RRC_5   5  // start trigger too weak   : strength_high <       rp.radio_sensitivity
#define RRC_6   6  // start trigger too strong : strength_high > 1.5 * rp.radio_sensitivity
#define RRC_7   7  // end of Reception during WARM_UP (cf. RRC_0, sender malfunction?)
#define RRC_8   8  // unreliable HIGH  during WARM_UP
#define RRC_9   9  // unreliable LOW   during WARM_UP
#define RRC_10 10  // less than three consecutive reliable signals (detected on HIGH)
#define RRC_11 11  // less than three consecutive reliable signals (detected on LOW)
#define RRC_12 12  // more than three consecutive unreliable signals (detected on HIGH)
#define RRC_13 13  // more than three consecutive unreliable signals (detected on LOW)
#define RRC_14 14  // more than three (reliable) consecutive collisions, or signal attenuation/loss
#define RRC_15 16  // program error
                                          
  // recorded signals : rs
  // ================
  // buffers: first index = 1 = index of the first HIGH, position 0 is not used
  // [odd indices] : HIGH-durations / HIGH-strengths 
  // [even indices]: LOW-durations  / LOW-strengths 
  typedef struct 
  {
    unsigned int *duration;   // signal_duration[NV + 5]      : (measured signal duration [polls]) / 2
    byte         *strength;   // signal_strength[WARM_UP + 1] : signal strength [dBm]  (WARM_UP >= 8 !!!)
    int          count;       // number of recorded signals (index of the last LOW before end-record)
    byte ref_strength_high;   // reference= (rs.strength[5] + rs.strength[7]) >> 1
    byte ref_strength_low;    // reference= (rs.strength[6] + rs.strength[8]) >> 1
    int  unreliable_count;    // total number of unreliable signals
  } recorded_signals;
  
  // receiver parameters : rp
  // ===================
  typedef struct 
  {
    byte radio_module;        // RM_1 or RM_2 depending on frequency to receive
    long radio_frequency;     // frequency
    byte radio_sensitivity;   // min strength to start reception (REG_OOKFIX)
    int  max_length;          // limitation on the number of signals to receive (count < limit)
  } receiver_parameters;
  
  byte recorder(receiver_parameters rp, recorded_signals &rs);

  // Poll Selected Radio (Wrapper)
  // ===================
  byte     loop_while_high  (unsigned int &duration_high, unsigned long &duration_low, byte &strength_low);
  byte     loop_while_low   (unsigned int &duration_high, unsigned long &duration_low, byte &strength_high, unsigned long duration_low_limit);  

  // Helper
  // ======
  byte rm1_loop_while_high  (unsigned int &duration_high, unsigned long &duration_low, byte &strength_low);
  byte rm1_loop_while_low   (unsigned int &duration_high, unsigned long &duration_low, byte &strength_high, unsigned long duration_low_limit);  
  byte rm2_loop_while_high  (unsigned int &duration_high, unsigned long &duration_low, byte &strength_low);
  byte rm2_loop_while_low   (unsigned int &duration_high, unsigned long &duration_low, byte &strength_high, unsigned long duration_low_limit);

  // initiate both radio modules to standby
  void init_radio();
  // setup SPI for both radio modules and set the 
  // _slaveSelectPin for the active radio module
  void SPI_begin(byte radio_module);
  // get the strength of a signal
  inline byte signal_strength();
  // wrappers: set ... of the active radio module
  void set_mode(byte mode);
  void set_frequency(long frequency);
  void set_threshold(byte threshold);
  void set_power(byte power_level);
