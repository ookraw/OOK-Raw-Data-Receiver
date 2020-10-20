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

  ===========================
  = Categorizer (Interface) =
  ===========================

// Off-Line Processing
// -------------------
   #include <iostream>
   #include <stdio.h>
   #include <stdlib.h>
   #include <stdint.h>
   using namespace std;
// include the following 6 lines in Eclipse (incl. F(x) !)
   #define F(x)  x
   void _psln (const char s[]) {printf("%s\n", s);}
   void _ps   (const char s[]) {printf("%s", s);}
   void _pdln (int d) {printf("%d\n", d);}
   void _pd   (int d) {printf("%d", d);}
   void _pc   (const char c) {printf("%c", c);}
*/

// Arduino print output  
#define _ps   Serial.print
#define _pd   Serial.print
#define _psln Serial.println
#define _pdln Serial.println
#define _pc   Serial.print

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// dimension of buffers
#define DIM_64    64  // dim uint16buf64
#define DIM_32    32  // dim uint8buf32
// dimension of the trace
#define NV  512       // number of signal durations (HIGH- plus LOW- duration values)
// dimensions < 256!
#define NC    8       // dim cluster       :  number of clusters
#define NA    8       // dim aggreg        :  number of aggregations
#define NO   16       // dim outlier_ind   :  number of outliers
#define NM   2*NO     // dim m_outlier_ind <= DIM_64 : number of merged HIGH- and LOW- outliers
#define NB   32       // dim bin_count     <= DIM_32 : number of bins per histogram
#define NH   2*NB     // dim h_hit_ind     <= DIM_64 : number of first bin-hits (if not enough memory: NH= 32

#define LSB   0b0000000000000001    // least significant bit  (v: reliability flag; v_ind: HIGH/LOW distinction)
#define MSB   0b1111111111111110    // most  significant bits
#define CEIL_U      65000U  // limit of HIGH / LOW durations (U: Unsigned, ~ 16 bits)
#define CEIL        65000   // limit of HIGH / LOW durations
//#define HIGH          1   // signal-HIGH index
//#define LOW           0   // signal-LOW  index
#define RELIABLE        0   // duration value is reliable   (see LSB)
#define UNRELIABLE      1   // duration value is unreliable (see LSB)
#define BORDER_WIDTH    8   // border width (warm-up / cool-down); number of required reliable start signals
#define START_VAL      50   // histogram: start value (first h_floor_val <= minimum raw data value)
#define MAX_HOLES       1   // histogram: tolerated number of empty bins within a cluster bin subsequence
#define FIRST_HITS      2   // histogram: the first 2 bin hits are recorded
#define MIN_SIZE        3   // histogram: minimum number of elements required to constitute a cluster
#define REL_DELTA      50   // relative delta per thousand (‰)
// classifier option
#define C_OPT_2 2     // relative delta: 25.00 %   outlier separation
#define C_OPT_3 3     // relative delta: 12.50 %
#define C_OPT_4 4     // relative delta:  6.25 %   test; resorber option

// categorizer return codes
// ========================
// consistency
#define CRC_0 0       // no error
#define CRC_1 1       // data inconsistency: checksum error
#define CRC_2 2       // data inconsistency: subsequence length error (only quadruples and quintuples are accepted)
// clusterability
#define CRC_3 3       // unclusterable: too many clusters
#define CRC_4 4       // unclusterable: too many aggregations
#define CRC_5 5       // unclusterable: too many outliers
#define CRC_6 6       // unclusterable: too many hits in histogram
#define CRC_7 7       // unclusterable: no cluster
#define CRC_8 8       // unclusterable: overlapping clusters
#define CRC_9 9       //
// fatal errors (program errors that should never occur)
#define CRC_10 10     // fatal error: histogram bin range error
#define CRC_11 11     // fatal error: bin_start_ind error error
#define CRC_12 12     // fatal error: very strange error
#define CRC_13 13     // fatal error: bin_stop_ind error
#define CRC_14 14     // fatal error: bins not empty error
#define CRC_15 15     // fatal error: number of outliers test error
#define CRC_16 16     // fatal error: merged outlier size error
#define CRC_17 17     // fatal error: aggregator error
#define CRC_18 18     // fatal error: resorber triple sum error

typedef struct {
  // clusters of trusted values each containing more than two values
  uint8_t  cluster_size;          // number of clusters
  uint16_t cluster_count[NC];     // number of raw data values included in this cluster (sum of of clustered bin frequencies)
  uint16_t cluster_ceil[NC];      // upper raw data limit (exclusive next bin following the last included bin)
  uint16_t cluster_center[NC];    // mean value of this cluster (approximation of the median)
  uint16_t cluster_floor[NC];     // lower raw data limit (inclusive first bin)
  // outlier indices
  uint8_t  outlier_size;          // number of outliers
  uint16_t outlier_ind[NO];       // indices of values that cannot be attributed to a cluster
  // aggregations of resistant outliers (containing a small number of values)
  uint8_t  aggreg_size_1;         // number of border-triggered clusters (level 1 aggregs that contain more than two values)
  uint8_t  aggreg_size_2;         // aggreg_size_1 + number of resistant outlier aggregations (level 2 aggregs of uncorrectable values)
  uint16_t aggreg_center[NA];     // mean value of this aggregation
  // separator barrier
  uint16_t separator_barrier;     // barrier between ordinary values and exceptionally big values (gap of factor ten)
  // inliers
  uint8_t  inlier_count;          // number of tolerated empty-bin-subsequences encountered within the bin sequence of a cluster
} categories;

// categorize signal durations into clusters of duration levels (HIGH/LOW processed separately)
int8_t categorizer (categories duration_category[], uint16_t signal_sequence[], uint16_t signal_count, uint16_t unreliable_count, uint8_t &error_code,
                    uint8_t uint8buf32[], uint16_t uint16buf64[]);

bool sequence_reader  (uint16_t signal_duration[], uint16_t &sequence_length, uint16_t &unreliable_count);
void clusterer        (categories &z,  uint16_t v[], uint16_t v_start_ind, uint16_t v_stop_ind, bool    &overlap_flag, uint8_t &rc, uint8_t uint8buf32[], uint16_t uint16buf64[]);
void corrector        (categories z[], uint16_t v[], uint16_t v_length, uint16_t unreliable_count, uint8_t &rc, uint16_t uint16buf64[]);
    bool extractor    (uint16_t   v[], uint16_t v_stop_ind, uint16_t &v_ind, uint16_t &ss_start_ind, uint16_t &ss_stop_ind);
    bool resorber     (categories &z,  uint16_t v[], uint16_t u[], uint16_t ss_start_ind, uint16_t ss_stop_ind, uint16_t &rel_delta, uint8_t &rc);
    void aggregator   (categories &z,  uint16_t v[], uint8_t  v_min_count, uint8_t &rc);
bool classifier       (categories &z,  uint16_t v_val, uint8_t &c_ind, uint16_t &c_val, uint8_t option);
void sequence_printer (categories z[], uint16_t v[], int16_t v_length);
void category_printer (categories &z,  uint16_t v[]);

void sort (uint16_t s[], uint16_t n);
void index_sort (uint16_t v[], uint16_t v_ind[], uint16_t n);
void merge (uint16_t a[], uint8_t na, uint16_t b[], uint8_t nb, uint16_t c[], uint8_t &nc);
void statistics (categories &z, uint16_t v[], uint16_t v_start_ind, uint16_t v_stop_ind);

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
