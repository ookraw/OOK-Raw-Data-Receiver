
#include <Arduino.h>
#include "categorizer.h"

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

  ==================
  = 2. Categorizer =  categorization of "continuous" signal durations into discrete duration levels
  ==================

  2.1 CLUSTERER: Histogram- & Post- Clustering
  2.1.1   Histogram-Clustering: based on histograms with adaptive bin sizes
  2.1.1.1 First Histogram Initialization
  2.1.1.2 Histogram Loop
  2.1.1.2.1 Bin Filling: discard both untrusted values and border values
  2.1.1.2.2 Bin Clustering
          - link bins that are separated by 1 empty bin at most
            - discard clusters that contain less than 3 values
            - in case of “overlapping clusters” abort the reception
  2.1.1.2.3 Outlier Sieving: collect the remaining values that could not be attributed to any cluster
  2.1.1.2.4 Next Histogram Initialization
  2.1.1.3 Test: Enumerate all Outliers
  2.1.2   Post-Clustering
  2.1.2.1 Border Processing: post-processing of the border values (discarded in histo-clustering)
  2.1.2.1.1 border values classification: if the classification fails, the value becomes an outlier
  2.1.2.1.2 border outlier aggregation: L1 aggregs
            aggregation of outlier values to “mini clusters” of at least 3 values;
            these aggregated outliers are removed from the list of outliers
  2.1.2.2 Cluster-Classification: identification of a separator barrier
          identification of a sufficiently large gap, separating ordinary values
          from top values, which will “by virtue of their size” be treated as reliable

  2.2 CORRECTOR: of Outliers and Untrusted Subsequences
  2.2.1   Outlier Correction: correction of reliable outliers identified by the clusterer
  2.2.1.1 reliable top-values preprocessing
          top-outliers, above the separator barrier, are aggregated to clusters; they remain in the list of outliers
  2.2.1.2 outlier separation
        - false outliers: can be corrected and attributed to a cluster; they are removed from the list of outliers
        - true  outliers: resist correction and will be aggregated (level 1 aggregs); they remain in the list of outliers
  2.2.1.3 resistant outlier aggregation
  2.2.2   Untrusted Subsequences Correction: correction of unreliable values identified by the recorder
  2.2.2.1 unreliable top-values preprocessing
          unreliable top-values, above the separator barrier, are added to the outliers and aggregated on the fly
  2.2.2.2 check for best-fit approximation
          the remaining values are approximated by the nearest cluster center:
          in absence of jumps, a “best-fit” is applied individually on each value of the untrusted subsequence
  2.2.2.3 check for jump elimination
          triplets comprising macro spikes or macro drops are resorbed:
          3 consecutive values are reduced to 1 value, followed by 2 zero durations

Trace driven Categorizer of OOK-Signals
=======================================
given     : a pulse sequence "TRACE" of alternating signal-HIGH and signal-LOW durations
objective : - identification of categories such that each duration value can be mapped to a corresponding category (duration levels)
            - error correction based on the identified categories, i.e. elimination of spikes, drops and outliers
principle steps:
- separate trusted data from subsequences that contain unreliable values which are discarded from clustering
- separate densely populated value ranges (-> clusters) from sparsely populated value ranges (-> outliers)
- separate resistant outliers from outliers which can be corrected and thereafter attributed to a cluster
- aggregate resistant outliers from outliers which can be corrected and thereafter attributed to a cluster
- correct and classify the remaining, untrusted data

TRACE
-----
value                : signal-HIGH or signal-LOW duration
                       HIGH- and LOW- durations are a priori unrelated and therefore clustered in separate steps
border values        : the first (warm-up) and last (cool-down) values of a sequence
             by construction (see recorder) the warm-up zone contains exclusively trusted values
reliable value       : both values involved in a transition are unreliable, if their signal strengths differs by less than 5 dBm
trusted value        : a value that is neither unreliable nor adjacent to an unreliable value
untrusted subsequence: subsequence of untrusted values
             an untrusted subsequence comprises either four or five values
             by construction (see recorder) they are separated by at least 3 consecutive reliable values

CATEGORIZER
-----------
clustering           : clustering of trusted, non-border values, based on histograms with adaptive bin sizes
 - clusters            cover dense value ranges that are populated by more than two values
                       a value x belongs to cluster ind, if : cluster_floor[ind] <= x < cluster_ceil[ind]
             clusters are settled once and for all (no additional or modified clusters after clustering)
 - outliers            cover sparsely populated value ranges that contain at most two values
other outlier sources: apart from clustering there are two additional sources of outliers:
 - border outliers     trusted border values that cannot be attributed to a cluster
 - top-outliers        special values above a rather high barrier (-> separator_barrier)
                       there are trusted and untrusted top-outliers, the latter is related to bizarre pulses ("spike followed by long pause")
separator_barrier    : separates big values which are an order of magnitude higher than ordinary values
                       the separator_barrier usually separates chained sequences
aggregations         : small clusters of aggregated border and outlier values
 - level 1 aggregs   : border-triggered aggregations, i.e. clusters that owe their existence to trusted border values (discarded from clustering)
 - level 2 aggregs   : aggregations of untrusted, correction-resistant outliers and
                       aggregations untrusted top-outliers
category             : the category of a value is an index that maps the value to a cluster or an aggregation
                       aggregation indices follow after the cluster indices
classifiable         : values that can be attributed to a category

OOK-Signals
-----------
On–Off Keying "OOK" is the modulation technique most widely found in low cost equipment:
 - a HIGH is sent by a full-power RF carrier
 - to transmit a LOW, the carrier is shut off
Information is conveyed by varying the duration of the HIGH- and LOW-signals.
In general, these durations are restricted to a limited number of duration levels (-> categories).

Clusterability
--------------
- nb of clusters (NC): <=  8, values map to relatively few clusters
- nb of aggregs  (NC): <=  8, values map to relatively few aggregations
- nb of outliers (NO): <= 16, the number of outliers is kept small
- nb of hits     (NH): <= 64, no restriction; max. 2 hits per bin (NB= 32 bins)

Robustness
----------
cluster robustness is enforced by the following measures:
- the two signal strength levels, HIGH and LOW, are separately clustered
- border values are kept away from clustering
- untrusted values are also discarded from clustering

*/

int8_t categorizer (                  // return code
  categories z[],                     // O  categories of raw data values  ([1]: HIGH-durations categories, [0]: LOW-durations categories)
  uint16_t   signal_duration[],       // IO signal sequence: [even indices]: HIGH-durations, [odd indices]: LOW-durations
  uint16_t   sequence_length,         // I  total number of signal durations: number of HIGH- plus LOW-durations
  uint16_t   unreliable_count,        // I  number of received unreliable (flagged) values contained in the signal sequence
  uint8_t    &return_code,            // O  return_code
  uint8_t    uint8buf32[],            // X uint8_t  buffer
  uint16_t   uint16buf64[]            // X uint16_t buffer
) {

  uint16_t sequence_start_ind;        // start index of signal_duration
  uint16_t sequence_stop_ind;         // stop  index of signal_duration
  bool     cluster_overlap;           // true, if at least one overlap between clusters has been detected
  cluster_overlap= false;

/*PP
  _psln(F(""));
  _psln(F("trusted HIGH-Values Clustering"));
  _psln(F("=============================="));
*/
  sequence_start_ind= 2 - HIGH;
  sequence_stop_ind=  sequence_length - HIGH;
  //P _ps(F("start index of HIGH-values: "));_pdln(sequence_start_ind);
  //P _ps(F("stop  index of HIGH-values: "));_pdln(sequence_stop_ind);
  clusterer (
    z[HIGH],             // O  signal_duration categories
    signal_duration,     // I  flagged raw data value sequence: odd indices: HIGH-durations, even indices: LOW-durations
    sequence_start_ind,  // I  start index of signal_duration
    sequence_stop_ind,   // I  stop  index of signal_duration
    cluster_overlap,     // O  true, if at least one overlap between clusters has been detected
    return_code,         // O  return_code  (CRC_0: no error)
    uint8buf32,
    uint16buf64
  );
  if (return_code != CRC_0) return (return_code);
/*PP
  category_printer (z[HIGH], signal_duration);
  statistics (
    z[HIGH],             // I  signal_duration categories
    signal_duration,     // I  flagged raw data value sequence: odd indices: HIGH-durations, even indices: LOW-durations
    sequence_start_ind,  // I  start index of signal_duration
    sequence_stop_ind    // I  stop  index of signal_duration
  );
*/
/*PP
  _psln(F(""));
  _psln(F("trusted LOW-Values Clustering"));
  _psln(F("============================="));
*/
  sequence_start_ind= 2 - LOW;
  sequence_stop_ind=  sequence_length - LOW;
  //P _ps(F("start index of LOW-values: "));_pdln(sequence_start_ind);
  //P _ps(F("stop  index of LOW-values: "));_pdln(sequence_stop_ind);
  clusterer (
    z[LOW],              // O  signal_duration categories
    signal_duration,     // I  flagged raw data value sequence: odd indices: HIGH-durations, even indices: LOW-durations
    sequence_start_ind,  // I  start index of signal_duration
    sequence_stop_ind,   // I  stop  index of signal_duration
    cluster_overlap,     // O  true, if at least one overlap between clusters has been detected
    return_code,         // O  return_code  (CRC_0: no error)
    uint8buf32,
    uint16buf64
  );
  if (return_code != CRC_0) return (return_code);
/*PP
  category_printer (z[LOW], signal_duration);
  statistics (
    z[LOW],              // I  signal_duration categories
    signal_duration,     // I  flagged raw data value sequence: odd indices: HIGH-durations, even indices: LOW-durations
    sequence_start_ind,  // I  start index of signal_duration
    sequence_stop_ind    // I  stop  index of signal_duration
  );
*/
/*PP
  _psln(F(""));
  _psln(F("Error Correction"));
  _psln(F("================"));
*/
  if (!cluster_overlap) {
    corrector (
      z,                    // IO signal_duration categories (clusters are not modified)
      signal_duration,      // I  flagged raw data value sequence: odd indices: HIGH-durations, even indices: LOW-durations
      sequence_length,      // I  number of signal durations: HIGH- plus LOW- durations without end markers
      unreliable_count,     // I  number of received unreliable (flagged) values contained in the signal sequence
      return_code,          // O  return_code  (CRC_0: no error)
      uint16buf64
    );
    if (return_code != CRC_0) return (return_code);
  }
/*PP
  _psln(F(""));
  _psln(F("HIGH-Value Categories"));
  _psln(F("====================="));
  category_printer (z[HIGH], signal_duration);
  _psln(F(""));
  _psln(F("LOW-Value Categories"));
  _psln(F("===================="));
  category_printer (z[LOW], signal_duration);
*/
  // duration_category
  _psln(F(""));
  _psln(F("Categorized Sequence"));
  //P _psln(F("===================="));
  sequence_printer (z, signal_duration, sequence_length);
  return (CRC_0);

} // end categorizer

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void clusterer (
  categories &z,          // O   result of clustering process (categories of either HIGH- (z[HIGH]) or LOW- durations (z[LOW]))
  uint16_t v[],           // I   flagged raw data value sequence: odd indices: HIGH-durations, even indices: LOW-durations
  uint16_t v_start_ind,   // I   start index of v[] (included)
  uint16_t v_stop_ind,    // I   stop  index of v[] (included)
  bool    &overlap_flag,  // O   true, if at least one overlap between clusters has been detected
  uint8_t &rc,            // O   return_code (0: no error)
  uint8_t  bin_count[],   // X   buffer: bin frequentation: number of values encountered in the range of the bin [b_ind] (0: empty; >0: occupied)
  uint16_t h_hit_ind[]    // X   buffer: - indices of those values that are the first to hit an empty / sparsely populated bin (h_count)
                          //             - with removed indices that are related to densely populated bins (h_count_2; -> outlier)
) {

  // ************* //
  // 2.1 CLUSTERER //  Histogram- & Post- Clustering
  // ************* //

  // 2.1.1   Histogram-Clustering: based on histograms with adaptive bin sizes
  // 2.1.1.1 First Histogram Initialization
  // 2.1.1.2 Histogram Loop
  // 2.1.1.2.1 Bin Filling
  //           discard both untrusted values and border values
  // 2.1.1.2.2 Bin Clustering
  //         - link bins that are separated by 1 empty bin at most
    //         - discard clusters that contain less than 3 values
    //         - in case of “overlapping clusters” abort the reception
  // 2.1.1.2.3 Outlier Sieving
  //           collect the remaining values that could not be attributed to any cluster
  // 2.1.1.2.4 Next Histogram Initialization
  // 2.1.1.3 Test: Enumerate all Outliers

  // 2.1.2   Post-Clustering
  // 2.1.2.1 Border Processing: post-processing of the border values (discarded in histo-clustering)
  // 2.1.2.1.1 border values classification
  //           classification of the border values; if the classification fails, the value becomes an outlier
  // 2.1.2.1.2 border outlier aggregation: L1 aggregs
  //           aggregation of outlier values to “mini clusters” of at least 3 values;
  //           these aggregated outliers are removed from the list of outliers
  // 2.1.2.2 Cluster-Classification: identification of a separator barrier
  //         identification of a sufficiently large gap, separating ordinary values
  //         from top values, which will “by virtue of their size” be treated as reliable


  // Clusterability Criteria
  // -----------------------
  // number of clusters (NC)  : <=  8, values map to relatively few clusters
  // number of aggregs  (NC)  : <=  8, values map to relatively few aggregations
  // number of outliers (NO)  : <= 16, the number of outliers is relatively small (e.g. < 5% of the data)
  // number of hits     (NH)  : <= 64, bin occupation rate <= 100%  (32: half of a histogram must be empty)

  // Cluster Robustness
  // ------------------
  // robust clusters are essential for error correction
  // the robustness is ensured by the following measures:
  // - the two signal strength levels, HIGH and LOW, are separately clustered
  // - border values are kept away from clustering
  // - unreliable values and their neighbors are also discarded from clustering
  // note that both values involved in a transition with a strength difference of less than 5 db are flagged as unreliable

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

{
  // +++++++++++++++++++++++++++ //
  // 2.1.1  Histogram-Clustering //  production of clusters & outliers
  // +++++++++++++++++++++++++++ //
  // 2.1.1.1 First Histogram Initialization
  // 2.1.1.2 Histogram Loop
  // 2.1.1.2.1 Bin Filling
  //           discard both untrusted values and border values
  // 2.1.1.2.2 Bin Clustering
  //         - link bins that are separated by 1 empty bin at most
    //         - discard clusters that contain less than 3 values
    //         - in case of “overlapping clusters” abort the reception
  // 2.1.1.2.3 Outlier Sieving
  //           collect the remaining values that could not be attributed to any cluster
  // 2.1.1.2.4 Next Histogram Initialization

  // current cluster
  uint8_t  c_ind;             // index of cluster
  uint16_t c_center;          // cluster mean value
  uint16_t c_count;           // sum of the frequencies of all bins that belong to the same cluster
  uint16_t c_prev_count;      // previous c_count
  uint8_t  c_hole_count;      // number of consecutive empty-bins encountered within the current cluster (MAX_HOLES)
  // current histogram
  uint32_t h_width;           // histogram range width: value range of the current histogram = NB * bin_width
  uint16_t h_floor_val;       // histogram base value : lowest value included in the current histogram
  uint16_t h_next_floor;      // floor of the next histogram (base value of the histogram with the next higher value range)
  uint16_t h_ceil_val;        // histogram ceil value : highest value =  h_floor_val + h_width (ceil is not included)
//  uint16_t h_hit_ind[NH];   // - indices of those values that are the first to hit an empty / sparsely populated bin (h_count)
                              // - with removed indices that are related to densely populated bins (h_count_2; -> outlier)
  uint8_t  h_ind;             // index of h_hit_ind[]
  uint8_t  h_count;           // number of elements in h_hit_ind[]
  // bins of current histogram   NB < 256: number of bins per histogram
  uint8_t  b_ind;             // index of bin : bin_count[b_ind]
//  uint8_t  bin_count[NB];   // bin frequentation: number of values encountered in the range of the bin [b_ind] (0: empty; >0: occupied)
  uint8_t  bin_start_ind;     // bin start index of cluster: to be mapped to z.cluster_floor[c_ind]
  uint8_t  bin_stop_ind;      // bin stop  index of cluster: to be mapped to z.cluster_ceil[c_ind]
  uint16_t bin_width;         // bin agglomeration width: value range per bin = 2 ** bin_width_2log
  uint8_t  bin_width_2log;    // 2log of bin_width
  uint32_t bin_mean;          // local mean value (bin level)
  // raw data values
  uint16_t v_ind;             // index of current value
  uint16_t v_val;             // current value = v[v_ind]
  uint16_t v_count;           // number of processed values (filtered and in range of v_start_ind ... v_stop_ind)

  bool outlier_presence_flag; // true, if the histogram contains at least one outlier
  bool ascending;             // used for overlap detection

  rc= CRC_0;
  // initialize cluster
  z.cluster_size=   0;
  z.aggreg_size_1=  0;
  z.aggreg_size_2=  0;
  z.outlier_size=   0;
  z.inlier_count=   0;
  c_ind= 0;

  // 2.1.1.1 First Histogram Initialization
  // **************************************
  // initialize first bottom value
  h_next_floor= START_VAL;
  // initialize bin size of histogram: BOTH bin_width AND corresponding 2log
  bin_width_2log= 4;
  bin_width= 1 << bin_width_2log;
  // initiate all bins to empty
  for (b_ind= 0; b_ind < NB; b_ind++) {
    bin_count[b_ind]= 0;
  }

  // 2.1.1.2 Histogram Loop
  // **********************
  // begin histogram main loop
  while (true) {
    // presence of at least one outlier in this histogram
    outlier_presence_flag= false;
    // histogram bin width
    // _psln("");
    // _ps(F("histogram bin_width  :"));_ps("\t");_pdln(bin_width);
    // histogram bin width (NB = number of bins in the histogram)
    h_width= NB * (uint32_t)bin_width;
    // _ps(F("histogram h_width    :"));_ps("\t");_pdln(h_width);
    // histogram value range
    // - histogram bottom value (floor is included)
    h_floor_val= h_next_floor;
    // _ps(F("histogram floor value:"));_ps("\t");_pdln(h_floor_val);
    // - histogram top value (ceil is excluded, h_width used as uint32_t temporary)
    h_width=  h_floor_val + h_width;
    if (h_width > CEIL_U) h_ceil_val= CEIL_U;
    else h_ceil_val= h_width;
    // _ps(F("histogram ceil  value:"));_ps("\t");_pdln(h_ceil_val);
    /*
    // test: all bins should be empty
    for (b_ind= 0; b_ind < NB; b_ind++) {
      if (bin_count[b_ind] > 0) {
        //E _psln(F("bins not empty error (should never occur !!!)"));
        rc= CRC_14;
        return;
      }
    }
    */
    // initiate the next histogram bottom value
    // find minimum: lowest value >= current top value
    h_next_floor= CEIL_U;

    // 2.1.1.2.1 Bin Filling: discard both untrusted values and border values
    // =====================
    // begin bin-filling (sequence scan, border values excluded)
    v_count= 0;
    // reset h_count: number of elements in h_hit_ind[]
    h_count= 0;
    // scan the trace between warm-up and cool-down
    for (v_ind= v_start_ind + BORDER_WIDTH; v_ind <= v_stop_ind - BORDER_WIDTH; v_ind+= 2) {
      // current value
      v_val= v[v_ind];
      // check range: floor value
      if (v_val <  h_floor_val) continue;
      // begin filter: check immediate neighborhood for unreliable values
        // - current element
        if ((v[v_ind]     & LSB) == UNRELIABLE) continue;
        // - element in front the current element
        if ((v[v_ind + 1] & LSB) == UNRELIABLE) continue;
        // - element at the back of the current element
        if ((v[v_ind - 1] & LSB) == UNRELIABLE) continue;
      // end filter
      // check range: ceil value
      // and determine floor of next round (after filter !!!)
      if (v_val >= h_ceil_val) {
        // floor value of next histogram = lowest filtered value above the current ceil value
        if (v_val < h_next_floor) h_next_floor= v_val;
        continue;
      }
      v_count++;
      // map value to bin
      b_ind= (v_val - h_floor_val) >> bin_width_2log;
      if (b_ind >= NB) {
        //E _psln(F("histogram bin range error (should never occur !!!)"));
        rc= CRC_10;
        return;
      }
      // maximum population per bin = 255 (size of byte)
      if (bin_count[b_ind] >= 255) continue;
      // add occurrence of the current value to the corresponding bin
      bin_count[b_ind]++;
      // record the first FIRST_HITS (indices of those values that first hit the bin)
      if (h_count < NH) {
        if (bin_count[b_ind] <= FIRST_HITS) h_hit_ind[h_count++]= v_ind;
      } else {
        // does not occur if NH >= 2*NB
        //E _ps(F("too many hits in histogram !!!"));_ps("\t");_pdln(h_count);
        rc= CRC_6;
        return;
      }
    }
    // end bin-filling
    // ---------------
    // _ps(F("h_count="));_ps("\t");_pdln(h_count);

/*PP
    // print histogram
    _ps(F("number of processed values:"));_ps("\t");_pdln(v_count);
    _psln("");
    _psln(F("histogram"));
    _psln(F("b_ind  value  count"));
    for (b_ind= 0; b_ind < NB; b_ind++) {
      _pd(b_ind);_ps("\t");_pd((b_ind << bin_width_2log) + h_floor_val);_ps("\t");_pdln(bin_count[b_ind]);
    }
    _psln("");
*/
    // 2.1.1.2.2 Bin Clustering
    // ========================
    // - link bins that are separated by 1 empty bin at most
    // - discard clusters that contain less than 3 values
    // - in case of “overlapping clusters” abort the reception
    // begin bin-clustering
    b_ind= 0;
    bin_stop_ind= 0;
    while (b_ind < NB) {
      // "START-BIN" of cluster interval: first occupied bin after a series of empty bins
      bin_start_ind= NB;
      while ((b_ind < NB) && (bin_count[b_ind++] == 0));
      // set start bin, it may be empty if b_ind == NB
      bin_start_ind= b_ind - 1;
      // check histogram end?
      if (b_ind >= NB) {
        // check histogram overlap: start bin adjacent to next higher histogram?
        if (bin_count[bin_start_ind] > 0) {
          // start bin is not empty
          // _psln(F("overlap: start bin adjacent to next histogram"));
          // let the next histogram take care of this cluster
          h_next_floor= (bin_start_ind << bin_width_2log) + h_floor_val;
          // to avoid double in the outlier list (this bin will be processed in the next histogram)
          bin_count[bin_start_ind]= 0;
        }
        break;  // continue after end of bin clustering
      }
      if (bin_start_ind >= NB) {
        //E _psln(F("bin_start_ind error error (should never occur !!!)"));
        rc= CRC_11;
        return;
      }

      // "STOP_BIN_SEQUENCE" of cluster interval: more than MAX_HOLES consecutive empty bins after a series of occupied bins
      // note that within the bin-sequence of a cluster at most MAX_HOLES consecutive empty bins are tolerated
      c_hole_count= 0;
      // no stop bin found
      bin_stop_ind= NB;
      // check histogram end?
      while (b_ind < NB) {
        // count number of consecutive empty bins
        if (bin_count[b_ind] > 0) {
          if (c_hole_count > 0) z.inlier_count++;
          c_hole_count= 0;
        } else {
          // number of consecutive empty bins > MAX_HOLES ?
          if (++c_hole_count > MAX_HOLES) {
            // set stop bin
            bin_stop_ind= b_ind - MAX_HOLES;
            break;
          }
        }
        b_ind++;
      }
      // check histogram end?
      if (b_ind == NB) {
        // _ps(F("==> bin_stop_ind : "));_ps("\t");_pdln(bin_stop_ind);
        // check overlap: stop bin sequence not yet reached?
        if (bin_stop_ind == NB) {
          // no stop bin found
          // _psln(F("overlap: no stop bin in this histogram"));
          // let the next histogram take care of this cluster
          h_next_floor= (bin_start_ind << bin_width_2log) + h_floor_val;
          // to avoid doubles in the outlier list (these bins will be processed in the next histogram)
          for (b_ind= bin_start_ind; b_ind < NB; b_ind++) {
            bin_count[b_ind]= 0;
          }
        } else {
          //E _psln(F("very strange error (should never occur !!!)"));
          rc= CRC_12;
          return;
        }
        break;  // continue after end of bin clustering
      }
      if (bin_stop_ind >= NB) {
        //E _psln(F("bin_stop_ind error (should never occur !!!)"));
        rc= CRC_13;
        return;
      }
/*PP
      _ps(F("bin_start_ind (incl.): "));_ps("\t");_pdln(bin_start_ind);
      _ps(F("bin_stop_ind  (excl.): "));_ps("\t");_pdln(bin_stop_ind);
*/
      // check overlapping clusters
      if (bin_stop_ind - bin_start_ind >= 6) {
        /*
        _psln(F("*** cluster interval >= 6 ***"));
        _psln(F("b_ind  value  count"));
        for (b_ind= bin_start_ind; b_ind < bin_stop_ind; b_ind++) {
          _pd(b_ind);_ps("\t");_pd((b_ind << bin_width_2log) + h_floor_val);_ps("\t");_pdln(bin_count[b_ind]);
        }
        */
        ascending= true;
        c_prev_count= 0;
        c_count= bin_count[bin_start_ind] + bin_count[bin_start_ind + 1];
        for (b_ind= (bin_start_ind + 2); b_ind < bin_stop_ind; b_ind++) {
          c_count+= bin_count[b_ind];
          if (ascending) {
            if ((c_count + 3) < c_prev_count) {   // NEU  MIN_SIZE ??
              // change to descending
              ascending= false;
            }
          } else {
            if (c_count > (c_prev_count + 3)) {   // NEU  MIN_SIZE ??
              // change to ascending
              _psln("");
              _psln(F("!!! overlapping clusters !!!"));
              overlap_flag= true;
              // rc= CRC_8;
              // return;

              ascending= true;
              bin_stop_ind= b_ind - 2;
              // _ps(F("bin_stop_ind  (excl.): "));_ps("\t");_pdln(bin_stop_ind);
              break;

            }
          }
          c_prev_count= c_count;
          // _pd(c_count);_ps("\t");
          c_count-= bin_count[b_ind - 2];
        }
        // _psln("");

      }

      // count number of elements in cluster and approximate the mean value
      c_count=  0;
      bin_mean= 0;
      uint8_t k= 1;
      for (b_ind= bin_start_ind; b_ind < bin_stop_ind; b_ind++) {
        c_count+= bin_count[b_ind];
        bin_mean+= k * bin_count[b_ind];
        k++;
      }

      // check number of elements in cluster
      if (c_count < MIN_SIZE) {
        // low density clusters contain less than three elements
        // these bins are not emptied -> outlier
        //P _ps(F("low density cluster: "));_ps("\t");_pdln(c_count);
        // set flag: the sequence contains at least one outlier
        outlier_presence_flag= true;
        continue;
      } else {
        // the bins of high density clusters are emptied to avoid confusion with outliers
        for (b_ind= bin_start_ind; b_ind < bin_stop_ind; b_ind++) bin_count[b_ind]= 0;
      }

      // record cluster
      // --------------
      z.cluster_count[c_ind]=  c_count;
      z.cluster_center[c_ind]= c_center=
               ((bin_start_ind << bin_width_2log) + ((bin_mean << bin_width_2log) / c_count) + h_floor_val - (bin_width >> 1)) & MSB;
      z.cluster_floor[c_ind]=  (bin_start_ind << bin_width_2log) + h_floor_val;
      z.cluster_ceil[c_ind]=   (bin_stop_ind  << bin_width_2log) + h_floor_val;

      if (++c_ind >= NC) {
        z.cluster_size= NC;
        //E _psln(F("too many clusters !!!"));
        rc= CRC_3;
        return;
      }
      b_ind= bin_stop_ind;
    }
    // end bin-clustering
    // ------------------

    // 2.1.1.2.3 Outlier Sieving: collect the remaining values that could not be attributed to any cluster
    // =========================
    // begin outlier collection
    if (outlier_presence_flag) {
      for (h_ind= 0; h_ind < h_count; h_ind++) {
        v_ind= h_hit_ind[h_ind];
        // current value
        v_val= v[v_ind];
        // map current value to bin
        b_ind= (v_val - h_floor_val) >> bin_width_2log;
        if (bin_count[b_ind] > 0) {
          // this bin belongs to an outlier aggregation, because
          // - bins belonging to high density clusters have been set to zero
          // - and also overlap bins have been set to zero
          //P _ps(F("new outlier"));_ps("\t");_pd(h_ind);_ps("\t");_pd(b_ind);_ps("\t");_pdln(v_val);
          if (z.outlier_size >= NO) {
            //E _psln(F("too many outliers !!!)"));
            rc= CRC_5;
            return;
          }
          z.outlier_ind[z.outlier_size++]= v_ind;
          bin_count[b_ind]--;
        }
      }
    }
    // end outlier collection
    // ----------------------
    /*
    // test: all bins should be emptied
    for (b_ind= 0; b_ind < NB; b_ind++) {
      if (bin_count[b_ind] > 0) {
        //E _psln(F("bins not empty error 2 (should never occur !!!)"));
        rc= CRC_14;
        return;
      }
    }
    */
/*PP
    if (z.outlier_size > 0) _psln(F("outliers (without borders)"));
    for (o_ind= 0; o_ind < z.outlier_size; o_ind++) {_ps("\t");_pd(z.outlier_ind[o_ind]);_ps("\t");_pdln(v[z.outlier_ind[o_ind]]);}
*/

    // 2.1.1.2.4 Next Histogram Initialization
    // =======================================

    // next value base
    // ---------------
    // _ps(F("next value base: "));_ps("\t");_pdln(h_next_floor);
    if (h_next_floor == CEIL_U) {
      break;
    }
    // move the next base in the middle of the first bin of the next histogram (subtract current bin_width)
    h_next_floor-= bin_width;

    // find appropriate bin_width and bin_width_2log for the next histogram
    // --------------------------------------------------------------------
    // temporary use of h_width (uint32_t)
    h_width= h_ceil_val;
    while (h_next_floor >= h_width) {
      bin_width_2log++;
      bin_width<<= 1;
      h_width+= NB * (uint32_t)bin_width;
      // _ps(F("new ceil: "));_ps("\t");_pd(h_width);_ps("\t");_pdln(NB * bin_width);
    }

  }
  // end histogram main loop
  // ***********************

  // number of clusters
  z.cluster_size= c_ind;
  if (z.cluster_size == 0) {
    //E _psln(F("no cluster !!!"));
    rc= CRC_7;
    return;
  }

/*
// begin test  ------------------------------------------------------------------------------------------------------------------------------------------------------
  h_count= 0;  // number of outliers
  for (v_ind= v_start_ind + BORDER_WIDTH; v_ind <= v_stop_ind - BORDER_WIDTH; v_ind+= 2) {
    // current value
    v_val= v[v_ind];
    // begin filter: check immediate neighborhood for unreliable values
      // filter: check immediate neighborhood for unreliable values
      // - element in front the current element
      if ((v_ind < v_stop_ind) && ((v[v_ind + 1] & LSB) == UNRELIABLE)) continue;
      // - current element
      if (((v[v_ind  ] & LSB) == UNRELIABLE)) continue;
      // - element at the back of the current element
      if ((v_ind > v_start_ind) && ((v[v_ind - 1] & LSB) == UNRELIABLE)) continue;
    // end filter: check immediate neighborhood for unreliable values
    // check whether the current value belongs to a cluster
    if (!classifier (z, v_val, c_ind, c_center, C_OPT_4)) {
      // _ps(F("TEST: "));_ps(_cT);_pd(v_ind);_ps(_cT);_pdln(v_val);
      h_count++;
    }
    if (h_count >= NO) break;
  }
  // _ps(F("TEST number of outliers: "));_ps(_cT);_pdln(h_count);
  if (z.outlier_size != h_count) {
    //E _psln(F("number of outliers test error (should never occur !!!)"));
    rc= CRC_15;
    return;
  }
// end test  --------------------------------------------------------------------------------------------------------------------------------------------------------
*/

}
// END Histogram-Clustering
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

{ // BEGIN Post-Clustering

  // +++++++++++++++++++++++ //
  // 2.1.2   Post-Clustering //  production of aggregations and outliers
  // +++++++++++++++++++++++ //
  // 2.1.2.1 Border Processing: post-processing of the border values (discarded in histogram clustering)
  // 2.1.2.1.1 border values classification
  //           classification of the border values; if the classification fails, the value becomes an outlier
  // 2.1.2.1.2 border outlier aggregation: L1 aggregs
  //           aggregation of outlier values to “mini clusters” of at least 3 values;
  //           these aggregated outliers are removed from the list of outliers
  // 2.1.2.2 Cluster-Classification: identification of a separator barrier
  //         identification of a sufficiently large gap, separating ordinary values
  //         from top values, which will “by virtue of their size” be treated as reliable
  // 2.1.2.3 Sort Outlier Indices


  // 2.1.2.1 Border Processing: post-processing of the border values (discarded in histo-clustering)
  // *************************
  // classification and correction of both border regions (warm-up and cool-down)
  // note: the first HIGH of the sequence is considered as too insecure to produce any outlier

  uint8_t  o_ind;             // index of outlier_ind
  uint8_t  ind;               // index of cluster
  uint16_t center;            // cluster mean value
  uint16_t v_ind;             // index of current value
  uint16_t v_val;             // current value = v[v_ind]
  uint16_t v_new_barrier;     // temporary separator_barrier
  uint16_t v_old_barrier;     // temporary separator_barrier

/*PP
  _psln("");
  _psln(F("border values"));
  _psln(F("-------------"));
  _psln(F("ind     orig    modified "));
*/

  // 2.1.2.1.1 border values classification
  // ======================================
  // classification of the border values; if the classification fails, the value becomes an outlier
  // (trusted values above the separator_barrier will also be included)
  for (v_ind= v_start_ind; v_ind <= v_stop_ind; v_ind+= 2) {
    // !!! => skip values between borders <=  !!!
    if (v_ind == v_start_ind + BORDER_WIDTH) v_ind= v_stop_ind - BORDER_WIDTH + 2;

    // current value
    v_val= v[v_ind];
    // begin filter: check immediate neighborhood for unreliable values
      // - current element
      if (((v[v_ind] & LSB) == UNRELIABLE)) continue;
      // - element in front the current element
      if ((v_ind < v_stop_ind)  && ((v[v_ind + 1] & LSB) == UNRELIABLE)) continue;
      // - element at the back of the current element
      if ((v_ind > v_start_ind) && ((v[v_ind - 1] & LSB) == UNRELIABLE)) continue;
    // end filter: check immediate neighborhood for unreliable values

    // check whether the current value belongs to a cluster
    // !!! use the same C_OPT as in sequence printer !!!
    if (classifier (z, v_val, ind, center, C_OPT_3)) {
      // original data
      //P _pd(v_ind);_ps("\t");_pd(v[v_ind]);
      // no modification of the raw data
      // $$$$$$$$$$$$$$$
      // v[v_ind]= center;
      // modified data
      //P _ps("\t");_pdln(v[v_ind]);
    } else {
      // the first HIGH of the sequence is too insecure to produce a useful outlier
      if (v_ind > 1) {
        // the nearest category is not near enough -> outlier
        //P _pd(v_ind);_ps("\t");_pd(v[v_ind]);_ps("\t");_psln(F("border outlier"));
        // the current value is an outlier, because it does not belong to any known cluster
        if (z.outlier_size >= NO) {
          //E _psln(F("too many outliers !!!)"));
          rc= CRC_5;
          return;
        }
        z.outlier_ind[z.outlier_size++]= v_ind;
      }
    }

  }
  //P _psln("");

/*PP
  if (z.outlier_size > 0) _psln(F("outliers (borders included)"));
  for (o_ind= 0; o_ind < z.outlier_size; o_ind++) {_ps("\t");_pd(z.outlier_ind[o_ind]);_ps("\t");_pdln(v[z.outlier_ind[o_ind]]);}
*/

  // 2.1.2.1.2 border outlier aggregation: L1 aggregs
  // ====================================
  // aggregation of outlier values to “mini clusters” of at least 3 values;
  // these aggregated outliers are removed from the list of outliers
  // L1 aggregs are clusters found after clustering because of the border zones

  // use same MIN_SIZE= 3 as in clustering
  aggregator (z, v, MIN_SIZE, rc);   // (L1 aggreg)
  z.aggreg_size_1= z.aggreg_size_2;
  if (rc > CRC_0) return;
  // eliminate aggregated outliers
  o_ind= 0;
  for (int k= 0; k < z.outlier_size; k++) {
    v_ind= z.outlier_ind[k];
    //P _pd(v_ind);_ps("\t");_pdln(v[v_ind]);
    // find the nearest cluster of v[v_ind]
    // !!! use the same C_OPT as in border values classification !!!
    if (!classifier (z, v[v_ind], ind, center, C_OPT_3)) {
      z.outlier_ind[o_ind++]= z.outlier_ind[k];
    }
  }
  z.outlier_size= o_ind;

  // 2.1.2.2 Cluster-Classification: identification of a separator barrier
  // ******************************
  // identification of a sufficiently large gap between or above outliers (separator_barrier), separating ordinary values
  // separating ordinary values from top values, which will “by virtue of their size” be treated as reliable
  v_old_barrier= 0;
  // initially the barrier is set to the highest cluster ceil
  v_new_barrier= z.cluster_ceil[z.cluster_size - 1];
  while (v_new_barrier > v_old_barrier) {
    v_old_barrier= v_new_barrier;
    v_new_barrier= 0;
    // z.separator_barrier == CEIL : the sequence has no top values
    // ensure: new z.separator_barrier <= CEIL
    if (v_old_barrier < (CEIL_U/10)) z.separator_barrier= 10 * v_old_barrier;
    else z.separator_barrier= CEIL_U;
    // _ps(F("!!! z.separator_barrier : "));_ps("\t");_pdln(z.separator_barrier);
    for (o_ind= 0; o_ind < z.outlier_size; o_ind++) {
      v_val= v[z.outlier_ind[o_ind]];
      if (v_val < z.separator_barrier) {
        // increase the separator_barrier
        if (v_val > v_new_barrier) v_new_barrier= v_val;
      }
      // else: v_val is an order of magnitude higher than everything met so far
      //       in this case the separator_barrier is not increased
    }
    // _ps(F("!!! v_new_barrier : "));_ps("\t");_pdln(v_new_barrier);
  }
  // _ps(F("separator_barrier:"));_ps("\t");_pdln(z.separator_barrier);

  // 2.1.2.3 Sort Outlier Indices
  // ****************************
  // allows later to merge outliers and process outlier pairs
  // this is the only place where sort is invoked (except statistics)
  sort(z.outlier_ind, z.outlier_size);
  /*PP
  _psln(F("outliers sorted:"));
  for (o_ind= 0; o_ind < z.outlier_size; o_ind++) {_pd(z.outlier_ind[o_ind]);_ps("\t");}_psln("");
  */

} // END Post-Clustering

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

}
// END CLUSTERER
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void corrector (
  categories z[],             // IO   result of clustering process (categories of 1: HIGH- and 2: LOW- durations)
  uint16_t v[],               // IO   flagged raw data value sequence: [odd indices]: HIGH-durations, [even indices]: LOW-durations
  uint16_t v_length,          // I    number of signal durations: HIGH- plus LOW- durations (without end markers)
  uint16_t unreliable_count,  // I    number of unreliable values in the sequence
  uint8_t &rc,                // O    return_code (0: no error)
  uint16_t m_outlier_ind[]    // X    buffer: merged outliers (merged HIGH- and LOW- outliers)
) {

  // ************* //
  // 2.2 CORRECTOR //  of Outliers and Untrusted Subsequences
  // ************* //

  // 2.2.1   Outlier Correction: correction of reliable outliers identified by the clusterer
  // 2.2.1.1 reliable top-values preprocessing
  //         top-outliers, above the separator barrier, are aggregated to clusters, but remain in the list of outliers
  // 2.2.1.2 outlier separation
  //       - false outliers: can be corrected and attributed to a cluster; they are removed from the list of outliers
  //       - true  outliers: resist correction and will be aggregated (level 1 aggregs); they remain in the list of outliers
  // 2.2.1.3 resistant outlier aggregation
  // 2.2.2   Untrusted Subsequences Correction: correction of unreliable values identified by the recorder
  // 2.2.2.1 unreliable top-values preprocessing
  //         unreliable top-values, above the separator barrier, are added to the outliers and aggregated on the fly
  // 2.2.2.2 check for best-fit approximation
  //         the remaining values are approximated by the nearest cluster center:
  //         in absence of jumps, a “best-fit” is applied individually on each value of the untrusted subsequence
  // 2.2.2.3 check for jump elimination
  //         triplets comprising macro spikes or macro drops are resorbed:
  //         3 consecutive values are reduced to 1 value, followed by 2 zero durations

  uint16_t v_start_ind; // start index of v[] (included)
  uint16_t v_stop_ind;  // stop  index of v[] (included)

  rc= CRC_0;
  if ((z[HIGH].cluster_size == 0) || (z[HIGH].cluster_size == 0)) {
    //E _psln(F("no cluster !!!)"));
    rc= CRC_7;
    return;
  }
  v_start_ind= 1;
  v_stop_ind=  v_length;

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // ++++++++++++++++++++++++ //
  // 2.2.1 Outlier Correction //  correction of reliable outliers identified by the clusterer
  // ++++++++++++++++++++++++ //
  // 2.2.1.1 reliable top-values preprocessing
  //         top-outliers, above the separator barrier, are aggregated to clusters, but remain in the list of outliers
  // 2.2.1.2 outlier separation
  //       - false outliers: can be corrected and attributed to a cluster; they are removed from the list of outliers
  //       - true  outliers: resist correction and will be aggregated (level 1 aggregs); they remain in the list of outliers
  // 2.2.1.3 resistant outlier aggregation

  if ((z[HIGH].outlier_size > 0) || (z[LOW].outlier_size > 0)) {

/*PP
    _psln("");
    _psln(F("Outlier Correction"));
    _psln(F("=================="));
*/
     int8_t  m_ind;         // index of merged outliers  (int8_t NOT uint8_t !)
//    uint16_t m_outlier_ind[NM]; // merged outliers (from HIGH and LOW)
    uint8_t  m_outlier_size;// merged outlier size
    uint8_t  cat_ind;       // index of current category (dummy parameter)
    uint16_t curr_center;   // cluster center of current value
    uint16_t prev_center;   // cluster center of preceding value
    uint16_t next_center;   // cluster center of following value
    int32_t  t_center_sum;  // sum of centers in the subsequence
    uint16_t curr_v_ind;    // index of current value
    uint16_t prev_v_ind;    // index of preceding value
    uint16_t next_v_ind;    // index of following value
    int32_t  v_sum;         // total sum of all values in the subsequence
    uint16_t rel_delta;     // the relative rel_delta (per thousand)
    uint16_t rel_delta_cor; // correctable rel_delta
    uint16_t rel_delta_max; // maximum relative delta during correction (trustworthiness of the final result)
    bool flag;

    rel_delta_max= 0;

    // merge HIGH and LOW outlier_ind[] (both are sorted!)
    // --------------------------------
    if ((z[HIGH].outlier_size + z[LOW].outlier_size) > NM) {
      //E _psln(F("merged outlier size error (should never occur !!!)"));
      rc= CRC_16;
      return;
    }
    merge (z[HIGH].outlier_ind, z[HIGH].outlier_size, z[LOW].outlier_ind, z[LOW].outlier_size, m_outlier_ind, m_outlier_size);
    if (m_outlier_size == 0) return;
///*PP
    _ps(F("outlier indices :"));
    for (m_ind= 0; m_ind < m_outlier_size; m_ind++) {
      _ps("\t");_pd(m_outlier_ind[m_ind]);
    }
    _psln("");
//*/
    _psln("");_psln(F("Outlier Correction"));
    //P _psln(F("ind     value"));

    // scan all outliers
    for (m_ind= m_outlier_size - 1; m_ind >= 0; m_ind--) {
      curr_v_ind= m_outlier_ind[m_ind];
      // _ps(F("curr_v_ind: "));_ps("\t");_pd(curr_v_ind);_ps("\t");_pdln(v[curr_v_ind]);

      // 2.2.1.1 reliable top-values preprocessing
      // *****************************************
      // top-outliers, above the separator barrier, are aggregated to clusters, but remain in the list of outliers

      // check if the value is ABOVE the separator_barrier
      // -------------------------------------------------
      if (v[curr_v_ind] >  z[curr_v_ind & LSB].separator_barrier) {
        // top-outlier, treated like a resistant outlier
        // ===========
        _ps(F("* top-outlier:"));_ps("\t");_pd(curr_v_ind);_ps("\t");_pdln(v[curr_v_ind]);
        continue;
      }

      // 2.2.1.2 outlier separation
      // **************************
      // - false outliers: can be corrected and attributed to a cluster; they are removed from the list of outliers
      // - true  outliers: resist correction and will be aggregated (level 1 aggregs); they remain in the list of outliers

      // outlier compensation with both preceding and following value
      // ------------------------------------------------------------
      flag= false;
      v_sum= v[curr_v_ind];
      t_center_sum= 0;
      // preceding value
      prev_v_ind= curr_v_ind - 1;
      if (prev_v_ind >= v_start_ind) {
        flag= classifier (z[prev_v_ind & LSB], v[prev_v_ind], cat_ind, prev_center, C_OPT_2);
        t_center_sum+= prev_center;
        v_sum+= v[prev_v_ind];
      }
      // following value
      next_v_ind= curr_v_ind + 1;
      if (next_v_ind <= v_stop_ind) {
        flag= classifier (z[next_v_ind & LSB], v[next_v_ind], cat_ind, next_center, C_OPT_2) && flag;
        t_center_sum+= next_center;
        v_sum+= v[next_v_ind];
      }
      flag= classifier (z[curr_v_ind & LSB], v[curr_v_ind], cat_ind, curr_center, C_OPT_2) || flag;
      // flag is true,  if the current outlier is classifiable OR both neighbors are classifiable

      // resistant outlier:
      rel_delta= ((int32_t)1000 * abs(v_sum - (t_center_sum + v[curr_v_ind]))) / v_sum;
      // correctable outlier:
      rel_delta_cor= ((int32_t)1000 * abs(v_sum - (t_center_sum + curr_center))) / v_sum;

      if ((!flag) || (rel_delta < rel_delta_cor)) {
        // resistant (true) outlier : the true, non-correctable outlier is saved for aggregation
        // ========================
        _ps(F("* resistant outlier:"));_ps("\t");_pd(curr_v_ind);_ps("\t");_pd(v[curr_v_ind]);_ps("\t");
        _ps(F("("));_pd(rel_delta);_ps(F(", "));_pd(rel_delta_cor);_ps(F(")"));_psln(F(" ‰"));
        continue;
      }
      // (the current outlier is classifiable OR both neighbors are classifiable)
      //  AND (corrected delta < delta without correction)

      // correctable (false) outlier : the false outlier is modified into a classifiable value and removed from the outliers
      // ===========================
      _ps(F("indices :"));_ps("\t");_pd(prev_v_ind);_ps("\t");_pd(curr_v_ind);_ps("\t");_pdln(next_v_ind);
      _ps(F("original:"));_ps("\t");_pd(v[prev_v_ind]);_ps("\t");_pd(v[curr_v_ind]);_ps("\t");_pdln(v[next_v_ind]);
      // modify raw data
      // $$$$$$$$$$$$$$$
      if (prev_v_ind >= v_start_ind) v[prev_v_ind]= prev_center & MSB;
      v[curr_v_ind]= curr_center & MSB;
      if (next_v_ind <= v_stop_ind) v[next_v_ind]= next_center & MSB;
      _ps(F("modified:"));_ps("\t");_pd(v[prev_v_ind]);_ps("\t");_pd(v[curr_v_ind]);_ps("\t");_pd(v[next_v_ind]);
      _ps("\t");_ps(F("("));_pd(rel_delta_cor);_psln(F(" ‰)"));
      if (rel_delta_cor > rel_delta_max) rel_delta_max= rel_delta_cor;
      // current: corrected outlier elimination
      // --------------------------------------
      m_outlier_ind[m_ind]= 0;

      // check if the preceding value is also an outlier
      if (m_outlier_ind[m_ind - 1] == prev_v_ind) {
        v[prev_v_ind]= prev_center;
        // adjacent: corrected outlier elimination
        // ---------------------------------------
        m_outlier_ind[m_ind - 1]= 0;
        m_ind--;
      }
    }
    // end scan outliers
    //P _psln("");

    _psln("");
    _ps(F("max. corr. rel. delta:"));_ps("\t");_pd(rel_delta_max);_psln(" ‰");


    // split (unmerge) the remaining, resistant outliers (non-correctable,)
    z[HIGH].outlier_size= 0;
    z[LOW].outlier_size=  0;
    for (m_ind= 0; m_ind < m_outlier_size; m_ind++) {
      curr_v_ind= m_outlier_ind[m_ind];
      if (curr_v_ind == 0) continue;
      z[curr_v_ind & LSB].outlier_ind[z[curr_v_ind & LSB].outlier_size++]= curr_v_ind;
    }

    // 2.2.1.3 resistant outlier aggregation  (L2 aggreg)
    // *************************************
    // outliers that resist correction are now recuperated as aggregations (L2 aggregs)
    // new outliers emerging from unreliable values are NOT recuperated,
    // only exception: untrusted top-outliers contained in untrusted subsequences
    // note: aggregator sorts outliers with respect to the values
/*PP
    _psln(F("Resistant Outlier Aggregation"));
    _psln(F("============================="));
*/
    if (rc > CRC_0) return;
    aggregator  (z[HIGH], v, 0, rc);   // (L2 aggreg)
    if (rc > CRC_0) return;
    aggregator  (z[LOW],  v, 0, rc);   // (L2 aggreg)

  }

// END Outlier Correction
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // +++++++++++++++++++++++++++++++++++++++++ //
  // 2.2.2   Untrusted Subsequences Correction //  correction of unreliable values identified by the recorder
  // +++++++++++++++++++++++++++++++++++++++++ //
  // 2.2.2.1 unreliable top-values preprocessing
  //         unreliable top-values, above the separator barrier, are added to the outliers and aggregated on the fly
  // 2.2.2.2 check for best-fit approximation
  //         the remaining values are approximated by the nearest cluster center:
  //         in absence of jumps, a “best-fit” is applied individually on each value of the untrusted subsequence
  // 2.2.2.3 check for jump elimination
  //         if tested positive, triplets comprising macro spikes or macro drops are resorbed:
  //         3 consecutive values are reduced to 1 value, followed by 2 zero durations

  // apart from untrusted top-outliers, there will be no new category

  if (rc > CRC_0) return;
  if (unreliable_count > 0) {
    _psln("");_psln(F("Untrusted Subsequences Correction"));

    uint16_t extractor_ind;   // extractor index (extractor controlled)
    uint16_t v_ind;           // index of current value
    uint16_t v_val;           // current value = v[v_ind]
    uint16_t ss_start_ind;    // first index of the subsequence of unreliable elements
    uint16_t ss_stop_ind;     // last  index of the subsequence of unreliable elements
    uint16_t ss_len;          // = ss_stop_ind - ss_start_ind + 1;
    uint16_t ss_cat[5];       // nearest category values of the local subsequence
    uint8_t  ss_ind;          // index of ss_cat
    uint8_t  cat_ind;         // index of current category (combined clusters and aggregations)
    uint16_t cat_val;         // value of current category (combined clusters and aggregations)
    int32_t  v_sum;           // total sum of all values in the subsequence
    int32_t  cat_sum;         // sum of category values in the subsequence
    uint16_t rel_delta;       // smaller relative delta obtained from the resorber and the best-fit approximation
    uint16_t rel_delta_max;   // maximum relative delta during correction (trustworthiness of the final result)
    bool flag;

    extractor_ind= v_start_ind + BORDER_WIDTH;
    rel_delta_max= 0;

    // extract the next untrusted subsequence
    // --------------------------------------
    while (extractor (v, v_stop_ind, extractor_ind, ss_start_ind, ss_stop_ind)) {
      _ps(F("indices :"));
      for (v_ind= ss_start_ind; v_ind <= ss_stop_ind; v_ind++) {
        _ps("\t");_pd(v_ind);
      }
      _psln("");

      ss_len= ss_stop_ind - ss_start_ind + 1;
      // tuple  of unreliable values: ss_len == 4
      // triple of unreliable values: ss_len == 5
      if ((ss_len < 4) || (ss_len > 5))  {
        //E _psln(F("subsequence length error (should never occur !!!)"));
        rc= CRC_2;
        return;
      }

      _ps(F("original:"));
      for (v_ind= ss_start_ind; v_ind <= ss_stop_ind; v_ind++) {
        _ps("\t");_pd(v[v_ind]);
      }
      _psln("");

      // 2.2.2.1 unreliable top-values preprocessing
      // *******************************************
      // unreliable top-values, above the separator barrier, are added to the outliers and aggregated on the fly
      // (a top-outlier, embedded in an untrusted subsequence, is considered reliable by sheer size)
      for (v_ind= ss_start_ind; v_ind <= ss_stop_ind; v_ind++) {
        // current value
        v_val= v[v_ind];

        // check if the value is ABOVE the separator_barrier
        if (v_val >  z[v_ind & LSB].separator_barrier) {
          // untrusted top-outlier
          // =====================
          _ps(F("* top-outlier:"));_ps("\t");_pd(v_ind);_ps("\t");_pdln(v_val);
          if (z[v_ind & LSB].outlier_size >= NO) {
            //E _psln(F("too many outliers !!!)"));
            rc= CRC_5;
            return;
          }
          // no attempt is made to correct a top-outlier: apply aggregation (L2 aggreg)
          z[v_ind & LSB].outlier_ind[z[v_ind & LSB].outlier_size++]= v_ind;
          //P _ps(F("new outlier    :"));_ps("\t");_pd(v_ind);_ps("\t");_pdln(v_val);
          // re-aggregate outliers
          aggregator (z[v_ind & LSB], v, 0, rc);   // (L2 aggreg)
        }
      }

      // 2.2.2.2 check for best-fit approximation
      // ****************************************
      // each value of the subsequence is approximated by the nearest category
      // ss_cat[]: the nearest category value of each value of the subsequence
      ss_ind=  0;
      v_sum=   0;
      cat_sum= 0;
      flag= true;
      for (v_ind= ss_start_ind; v_ind <= ss_stop_ind; v_ind++) {
        // find the nearest category of the current value
        // and check whether the current value is classifiable
        flag= classifier (z[v_ind & LSB], v[v_ind], cat_ind, cat_val, C_OPT_3) && flag;
        v_sum+= v[v_ind];
        cat_sum+= cat_val;
        ss_cat[ss_ind++]= cat_val;
      }
      rel_delta= ((int32_t)1000 * abs(v_sum - cat_sum)) / v_sum;
      // _ps(F("best-fit rel_delta: "));_ps("\t");_pd(rel_delta);_psln(" ‰");

      if (flag) {
//        _psln(F("######################################"));
        // all values are classifiable
        // ===========================
        // modify raw data (thereby removing the reliability flag)
        // $$$$$$$$$$$$$$$
        ss_ind= 0;
        for (v_ind= ss_start_ind; v_ind <= ss_stop_ind; v_ind++) {
          v[v_ind]= ss_cat[ss_ind++];
        }
      } else {
        // 2.2.2.3 check for jump elimination
        // **********************************
        // triplets comprising macro spikes or macro drops are resorbed:
        // (triplet resorption: spike/drop elimination)
        // (the central triple starts at ss_start_ind + 1)
        if (resorber (z[(ss_start_ind + 1) & LSB], v, ss_cat, ss_start_ind, ss_stop_ind, rel_delta, rc)) {
          // jump elimination (spike / drop)
          // ================
          // if the resorber gives a better result than the best-fit approximation,
          // the raw data is modified within the resorber
        } else {
          // resorber error: leave corrector
          if (rc > CRC_0) return;
          // best-fit approximation gives a better result than the resorber
          // ======================
          // modify raw data (thereby removing the reliability flag)
          // $$$$$$$$$$$$$$$
          ss_ind= 0;
          for (v_ind= ss_start_ind; v_ind <= ss_stop_ind; v_ind++) {
            v[v_ind]= ss_cat[ss_ind++];
          }
        }
      }

      // maximum rel_delta during correction
      if (rel_delta > rel_delta_max) rel_delta_max= rel_delta;

      _ps(F("modified:"));
      for (v_ind= ss_start_ind; v_ind <= ss_stop_ind; v_ind++) {
        _ps("\t");_pd(v[v_ind]);
      }
      _ps("\t");_ps("(");_pd(rel_delta);_psln(" ‰)");
    }
    _psln("");
    _ps(F("max. rel. delta:"));_ps("\t");_pd(rel_delta_max);_psln(" ‰");
  }

// END Untrusted Subsequences Correction
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}
// END CORRECTOR
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
