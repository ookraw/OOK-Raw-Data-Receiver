// !!! the first HIGH is at index position 1 (position 0 is not used) !!!
// !!! recorded rs.duration = (measured duration) / 2                 !!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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

  ===============
  = 1. Recorder =
  ===============

  1. Recorder
  1.1 start receiver
  1.2 detect begin of reception
  1.3 process warm-up signals
  1.4 prepare reception of following signals
  1.5 record following signals
  1.6 detect end of reception
  1.6.1 process nomal end
  1.6.2 process forced end 

  TRACE:
  - rs.count is without end-record, it is the index of the last LOW
  - end-record is either a pause (x, CEIL) or a zero duration (0, 0) 
    x is the last HIGH after the last LOW
  - the following records will be printed after the end of the trace (see reporting):  
    - checkout record   : (checksum, rs.unreliable_count)
    - end-of-data record: (-1, -1)
  
*/

#include <Arduino.h>
#include <SPI.h>
#include "radio_lib.h" 
 
byte recorder(receiver_parameters rp, recorded_signals &rs) 
{   
  // record OOK raw data
  // -------------------
  // rp       : input  only  :   reception parameters
  // signal   : output only  :   signal durations and strengths (strengths for first WARM_UP signals only)
  // returns  : ret_code    0:   end reached; 1: limit reached; >1: aborted 
  //                      6-9:   start; 10-13: unreliable; 14: collision/loss (cf. radio_lib.h)
  // reception start criteria: - after a sufficiently long pause (LONG_PAUSE)
  //                           - followed by a sufficiently strong signal (rp.radio_sensitivity)
  // reception end   criteria: - sufficiently long pause (duration_low_limit= LONG_PAUSE) OR 
  //                           - number of received signals (rs.count >= rp.max_length) OR 
  //                           - reception aborted (rs.count includes last reliable LOW)
  // noise    : - 2 or 3 consecutive unreliable signals are acceptable (usually repairable)
  //              if followed by at least three consecutive reliable signals 
  //            - the reception ends (return code RRC_14) if more than three (reliable) consecutive collisions, 
  //              or a signal attenuation/loss is detected (same return code for collision and signal loss)

  // note     : - rs.count is without end-record, it is the index of the last LOW
  //            - end-record is either a pause (x, CEIL) or a zero duration (0, 0) 
  //              x is the last HIGH after the last LOW

  unsigned int  duration_high;        // duration of the HIGH signal [poll-cycles] 
  unsigned long duration_low;         // duration of the LOW  signal [poll-cycles] 
  unsigned long duration_low_limit;   // LOW "timeout" duration -> end of reception
  byte strength_high;
  byte strength_low;
  byte curr_strength_high;
  byte prev_strength_low;
  byte cons_reliable_count;        // count of consecutive reliabel signals after the last unreliabel signal
  byte cons_unreliable_count;      // count of consecutive unreliabel signals after the last reliabel signal
  byte cons_collision_count;       // count of reliabel signals with significant difference of strength
  byte ret_code;
  int  ind;   
  byte strength_upper_lim;
  byte strength_lower_lim;
 
  // ****************** //
  // 1.1 start receiver //
  // ****************** //
  // setup SPI for both radio modules and
  // set the _slaveSelectPin for the active radio 
  SPI_begin(rp.radio_module);
  // set frequency 
  set_frequency(rp.radio_frequency);
  // set threshold (double of sensitivity in dBm)
  // +++++++++++++ 
  strength_high= 2 * rp.radio_sensitivity;
  set_threshold(strength_high);
  // set power
  set_power(MAX_POWER);
  // set mode RX
  set_mode(RF69_MODE_RX);
  delayMicroseconds(100);
  // set number of received signals
  rs.count= 1;  // not zero!
  rs.unreliable_count=  0;

  // ***************************** //
  // 1.2 detect begin of reception //   first HIGH after a LONG_PAUSE
  // ***************************** //
  // return codes:
  // RRC_0 : duration_low >= LONG_PAUSE, end of Reception
  // RRC_1 : end of HIGH / end of LOW / end of buffer (limit= NV)
  
  // wait for a long pause
  // ---------------------
  duration_low_limit= LONG_PAUSE;
  duration_low= 0;
  while (RRC_0 != loop_while_low(duration_high, duration_low, strength_high, duration_low_limit));   // LOW   <------------------
  // ret_code is equal to RRC_0: the loop is left with an ongoing long pause (signal is still LOW)

  // wait for the end of the ongoing long pause == wait for the start signal
  // ------------------------------------------
  duration_low_limit= INFINITE_PAUSE;
  duration_low= 0;
  while (RRC_1 != loop_while_low(duration_high, duration_low, strength_high, duration_low_limit));   // LOW   <------------------ 
  // ret_code is equal to RRC_1: the LOW has now ended, the reception start is detected 
  
  // the LOW has ended: strength_high = strength of start trigger (first HIGH)
  // -----------------
  rs.ref_strength_high= strength_high;
  rs.ref_strength_low=  0; 
  // check the strength of the start signal
  if (strength_high < rp.radio_sensitivity) {
    // start trigger too weak
    ret_code= RRC_5;
    goto EOR;                                                                                  // EOR   --->  start trigger too weak
  }
  if (strength_high > (rp.radio_sensitivity + (rp.radio_sensitivity >> 1))) {
    // start trigger too strong
    ret_code= RRC_6;
    goto EOR;                                                                                  // EOR   --->  start trigger too strong
  }
  
  // initialize 
  rs.ref_strength_high= 0;
  rs.ref_strength_low=  0; 
  duration_low_limit= LONG_PAUSE;
  strength_low= 0;
  // the first HIGH is at index position 1 (position 0 is not used)
  ind= 1; 

  // *************************** //
  // 1.3 process warm-up signals //
  // *************************** //
  // first 8 signals during WARM_UP must be reliable 
  
  while(ind <= WARM_UP) {
    // odd indices: HIGH
    // =================
    rs.strength[ind]= strength_high;
    // strength of curent high originates from pevious loop_while_low
    curr_strength_high= strength_high;
    prev_strength_low=  strength_low;
    ret_code= loop_while_high(duration_high, duration_low, strength_low);                      // HIGH  <------------------------    
    if (ret_code != RRC_1) {
      rs.duration[ind++]= (duration_high >> 1) | LSB;
      goto EOR;
    }
    // current HIGH duration = duration_high;
 
    // check edge in front of HIGH
    if ( strength_high >= (prev_strength_low + DELTA_STRENGTH)
      // check edge after HIGH
      && strength_high >= (strength_low + DELTA_STRENGTH)) {
        // mark HIGH "reliabel" (reset least significant bit)
        rs.duration[ind++]= (duration_high >> 1) & MSB;
    } else {
      // mark HIGH "unreliabel" (set least significant bit)
      rs.duration[ind++]= (duration_high >> 1) | LSB;  
      // unreliable HIGH during WARM_UP
      ret_code= RRC_8;
      goto EOR;
    }

    // even indices: LOW
    // =================
    rs.strength[ind]= strength_low;
    ret_code= loop_while_low(duration_high, duration_low, strength_high, duration_low_limit);  // LOW   <------------------------
    // (duration_high and strength_high concern the next signal, NOT the current one)
    if (ret_code != RRC_1) {
      if (ret_code == RRC_0) {
        // end of reception during WARM_UP
        rs.duration[ind++]= (duration_low >> 1) & MSB;
        // sender malfunction?
        ret_code= RRC_7; 
      } else {
        rs.duration[ind++]= (duration_low >> 1) | LSB;          
      }
      rs.count= ind - 2; 
      goto EOR;
    }
    
    // check edge in front of LOW
    if (curr_strength_high >= (strength_low + DELTA_STRENGTH)
      // check edge after LOW
      && strength_high >= (strength_low + DELTA_STRENGTH)) {
        // mark LOW "reliabel" (reset least significant bit)
        rs.duration[ind++]= (duration_low >> 1) & MSB;
    } else {
      // mark LOW "unreliabel" (set least significant bit)
      rs.duration[ind++]= (duration_low >> 1) | LSB;  
      // unreliable LOW during WARM_UP
      ret_code= RRC_9;
      goto EOR;
    }
  } // end WARM_UP

  // ****************************************** //
  // 1.4 prepare reception of following signals //
  // ****************************************** //
  // initialize consecutive signals counters
  cons_collision_count=  0;  
  cons_unreliable_count= 0;
  cons_reliable_count=   WARM_UP;
  // set reference strengths 
  // !!! WARM_UP >= 8 !!!
  rs.ref_strength_high= (rs.strength[5] + rs.strength[7]) >> 1;
  rs.ref_strength_low=  (rs.strength[6] + rs.strength[8]) >> 1; 
  /*
  // set adapted threshold (sensitivity in dBm)
  // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
  set_threshold(rs.ref_strength_high + rs.ref_strength_low);
  duration_high+= 25;
  */
  // set boundaries for collision detection
  strength_upper_lim= rs.ref_strength_high + DELTA_STRENGTH;
  strength_lower_lim= rs.ref_strength_high - DELTA_STRENGTH;

  
  // **************************** //
  // 1.5 record following signals //
  // **************************** //
  do { 
    // odd indices: HIGH
    // =================
    // strength of current high originates from pevious loop_while_low
    curr_strength_high= strength_high;
    prev_strength_low=  strength_low;
    ret_code= loop_while_high(duration_high, duration_low, strength_low);                      // HIGH <------------------------
    if (ret_code != RRC_1) {
      // basic reception error
      rs.duration[ind++]= (duration_high >> 1) | LSB;
      goto EOR;
    }
    // current HIGH duration = duration_high;

    // check edge in front of HIGH
    if ( strength_high >= (prev_strength_low + DELTA_STRENGTH)
      // check edge after HIGH
      && strength_high >= (strength_low + DELTA_STRENGTH)) {
        // mark HIGH "reliabel" (reset least significant bit)
        rs.duration[ind++]= (duration_high >> 1) & MSB;
        // rs.count= ind; only set on low
        // cons_reliable_count++;
        if (cons_reliable_count < 3) cons_reliable_count++;

        cons_unreliable_count= 0;       
        // collision detection
        if ((curr_strength_high > strength_upper_lim) || (curr_strength_high < strength_lower_lim)) {
          if (++cons_collision_count > 3) {
            // more than three (reliable) consecutive collisions, or signal attenuation/loss
            ret_code= RRC_14;
            goto EOR;
          }
        } else {
            cons_collision_count= 0;        
        }
    } else {
      // mark HIGH "unreliabel" (set least significant bit)
      rs.duration[ind++]= (duration_high >> 1) | LSB;  
      rs.unreliable_count++;
      if (cons_unreliable_count == 0) {
        if (cons_reliable_count < 3) {
          // less than three consecutive reliable signals (detected on HIGH)
          ret_code= RRC_10;
          goto EOR;          
        }        
      }
      cons_reliable_count= 0;
      if (++cons_unreliable_count > 3) {
        // more than three consecutive unreliable signals (detected on HIGH)
        ret_code= RRC_12;
        goto EOR;
      }     
    }
   
    // even indices: LOW
    // =================
    ret_code= loop_while_low(duration_high, duration_low, strength_high, duration_low_limit);  // LOW   <------------------------
    // (duration_high and strength_high concern the next signal, NOT the current one)
    if (ret_code != RRC_1) {
      if (ret_code == RRC_0) {
        
        // *********************** //
        // 1.6.1 process nomal end //
        // *********************** //
        // the last LOW is sufficiently long: duration_low == PAUSE

        rs.duration[ind++]= (duration_low >> 1) & MSB;
        // cut the ending pause
        rs.count= ind - 2;       
        // set active radio to standby
        set_mode(RF69_MODE_STANDBY);
        SPI.end(); 
        rs.count--;
        return ret_code;                                                                       // EOR   --->  return after pause
      } else {
        rs.duration[ind++]= (duration_low >> 1) | LSB; 
      }
      goto EOR;                                                                                // EOR   --->  end after error
    }

    // check edge in front of LOW
    if (curr_strength_high >= (strength_low + DELTA_STRENGTH)
      // check edge after LOW
      && strength_high >= (strength_low + DELTA_STRENGTH)) {
        // mark LOW "reliabel" (reset least significant bit)
        rs.duration[ind++]= (duration_low >> 1) & MSB;
        rs.count= ind;
        if (cons_reliable_count < 3) cons_reliable_count++;
        cons_unreliable_count= 0;
    } else {
      // mark LOW "unreliabel" (set least significant bit)
      rs.duration[ind++]= (duration_low >> 1) | LSB;  
      rs.unreliable_count++;    
      if (cons_unreliable_count == 0) {
        if (cons_reliable_count < 3) {
          // less than three consecutive reliable signals (detected on LOW)
          ret_code= RRC_11;
          goto EOR;          
        }        
      }
      cons_reliable_count= 0;
      if (++cons_unreliable_count > 3) {
        // more than three consecutive unreliable signals (detected on LOW)
        ret_code= RRC_13;
        goto EOR;
      }     
    } 
  } while (ind < rp.max_length);
  // end of Buffer (NV) or limitation (option)
  ret_code= RRC_1;
  rs.count= ind;

  // ************************ //
  // 1.6.2 process forced end //
  // ************************ //
  // - the reception limit is reached or
  // - the reception was aborted
EOR:
  // add two zeros, similar as pause: (0, CEIL)
  rs.duration[rs.count]= rs.duration[rs.count+1]= 0;
  // set active radio to standby
  set_mode(RF69_MODE_STANDBY);
  SPI.end();
  rs.count--;
  return ret_code;  
}
