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

  ==========================
  = 3. Categorizer Library =
  ==========================

  3.1 extractor : extract the next subsequence of untrusted values
  3.2 resorber  : resorb spikes and drops
  3.3 aggregator: aggregate border outliers (L1), resistant outliers (L2) and untrusted top-outliers (L2)
  3.4 classifier: find the nearest category (comprising clusters and aggregations)
  3.5 sequence_printer: map the raw data into a categorized sequence (category indices)
  3.6 category_printer: print the categories (clusters and aggregations)
  3.7 Helper
  3.7.1 sort: insertion sort (ascending)
  3.7.2 index_sort: index insertion sort (ascending)
  3.7.3 merge: merging of sorted arrays (without doubles)
  3.7.4 statistics: compute mean, median and absolute deviation
  3.7.5 miscellaneous

*/
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool extractor (           //     return true, if a valid subsequence has been found
  uint16_t   v[],          // I   flagged raw data value sequence: odd indices: HIGH-durations, even indices: LOW-durations
  uint16_t   v_stop_ind,   // I   stop  index of v[] (included)
  uint16_t  &v_ind,        // IO  current extractor index position in the array of values (scan progress)
  uint16_t  &ss_start_ind, // O   first index of the subsequence of unreliable elements
  uint16_t  &ss_stop_ind   // O   last  index of the subsequence of unreliable elements
) {
  // ************* //
  // 3.1 extractor //  extract next untrusted subsequence
  // ************* //
  // extract the next subsequence of untrusted values (those skipped during clustering)
  // start the subsequence with the element in front of the next unreliable value
  // end   the subsequence with the element after the last unreliable value
  // note:
  // - v_ind >= 2, because v_ind starts at BORDER_WIDTH + 1
  // - the subsequence length is intentionally not checked here but later in the resorber
  // - a reliable element must be found before or at v_stop_ind

  ss_start_ind= 0;
  ss_stop_ind=  0;

  // find the next start index of the subsequence
  for ( ; v_ind <= v_stop_ind - 2; v_ind++) {
    if ((v[v_ind] & LSB) == UNRELIABLE) {
      ss_start_ind= v_ind - 1;
      goto FIND_STOP_INDEX;
    }
  }
  return false;

FIND_STOP_INDEX:
  // find the stop index of the subsequence
  for ( ; v_ind <= v_stop_ind; v_ind++) {
    if (((v[v_ind] & LSB) == RELIABLE)) {
      // the current element is reliable
      ss_stop_ind= v_ind;
      v_ind++;
      return true;
    }
  }
  return false;
}
// END extractor

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool resorber (              //    return true on successful correction of a spike or a drop
    categories &z,           // I  categories of the central triple (either HIGH- or LOW- raw data values)
    uint16_t  v[],           // IO flagged raw data value sequence: odd indices: HIGH-durations, even indices: LOW-durations
    uint16_t  ss_cat[],      // I  category values (centers) of the subsequence
    uint16_t  ss_start_ind,  // I  start index of the quintuple
    uint16_t  ss_stop_ind,   // I  stop  index of the quintuple
    uint16_t &rel_delta,     // IO I: best-fit; O: smaller relative delta of resorber and best-fit
    uint8_t  &rc             // O  return_code (0: no error)
) {
  // ************ //
  // 3.2 resorber //   resorb spikes and drops
  // ************ //
  // examine the triple embedded within the quintuple for an occurrence of a spike or a drop
  // - the triple must be classifiable
  // - the rel_delta of the resorber must be smaller than the rel_delta of the best-fit approximation
  //   (rel_delta: relative delta (per thousand) between the sum of original values and the sum of the modified values)

  uint16_t v_ind;       // index of values v
  int32_t  v_sum;       // sum of triple plus contributions from front and back values
  uint16_t triple_val;  // value corresponding to v_sum (= (uint16_t) v_sum)
  int32_t  cat_sum;     // sum of centers in the subsequence
  uint8_t  cat_ind;     // index of current category (combined clusters and aggregations)
  uint16_t cat_val;     // value of current category (combined clusters and aggregations)
  uint16_t rel_delta_bestfit;  // relative delta of the best-fit approximation (rel_delta on input)
  uint8_t  option;      // tightness: relative distance between value and category center (cluster/aggregation)

  rc= CRC_0;
  // subsequence length must be equal to 5
  if ((ss_stop_ind - ss_start_ind) != 4) {
    // _ps(F("reass sub len: "));_ps("\t");_pd(ss_start_ind);_ps("\t");_pdln(ss_stop_ind);
    // only best-fit is applicable
    return (false);
  }

  // initialize
  rel_delta_bestfit= rel_delta;
  if (rel_delta_bestfit > 100) option= C_OPT_3;  // (12.5 %)
  else option= C_OPT_4;  // (6.25 %)
  v_ind= ss_start_ind;

  // check whether the central triple is classifiable
  // build triple sum including fractions from front and back element
  // contribution of element in front
  v_sum=  (int32_t)v[v_ind] - (int32_t)ss_cat[0];
  // contribution of the triple under test
  v_sum+= v[v_ind+1] + v[v_ind+2] + v[v_ind+3];
  // contribution of the element at the back
  v_sum+= (int32_t)v[v_ind+4] - (int32_t)ss_cat[4];
  if (v_sum > CEIL) {
    // fatal error: resorber triple sum error
    rc= CRC_18;
    // only best-fit is applicable
    return (false);
  }
  // sum of triple plus contributions of direct neighbors
  triple_val= (uint16_t) v_sum;
  // check whether the triple value is classifiable
  if (!classifier (z, triple_val, cat_ind, cat_val, option)) {
    // _ps(F("resorber triple_val not classifiable: "));_ps("\t");_pdln(triple_val);
    // only best-fit is applicable
    return (false);
  }
  // the nearest category is near enough to the resorbed value

  // rel_delta per thousand between the sum of values (v_sum) and the sum of nearest categories (cat_sum)
  v_sum= 0;
  for (v_ind= ss_start_ind; v_ind <= ss_stop_ind; v_ind++) v_sum+= v[v_ind];
  cat_sum= ss_cat[0] + cat_val + ss_cat[4];
  rel_delta= ((int32_t)1000 * abs(v_sum - cat_sum)) / v_sum;
  // _ps(F("resorber rel_delta: "));_ps("\t");_pd(rel_delta);_psln(" ‰");
  if (rel_delta > rel_delta_bestfit) {
    // best-fit gives smaller rel_delta compared to resorber
    rel_delta= rel_delta_bestfit;
    return (false);
  }

  // resorber gives smaller rel_delta compared to best-fit approximation
  // modify the raw data
  // $$$$$$$$$$$$$$$$$$$
  v_ind= ss_start_ind;
  // correct front element
  v[v_ind]= ss_cat[0];
  // handle spike/drop
  v[v_ind+1]= cat_val;
  v[v_ind+2]= 0;
  v[v_ind+3]= 0;
  // correct back element
  v[v_ind+4]= ss_cat[4];

  // cluster or aggregation?
  if (cat_ind >= z.cluster_size) {
    // add new outlier (because each outlier index must be recorded!)
    if (z.outlier_size >= NO) {
      //E _psln(F("too many outliers !!!)"));
      rc= CRC_5;
      return (false);
    }
    z.outlier_ind[z.outlier_size++]= ss_start_ind + 1;
  }

  return (true);

}
// END resorber

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void aggregator (
    categories &z,          // I  categories of the central triple (either HIGH- or LOW- raw data values)
    uint16_t v[],           // IO flagged raw data value sequence: odd indices: HIGH-durations, even indices: LOW-durations
    uint8_t v_min_count,    // I  required minimum number of elements (MIN_SIZE)
    uint8_t &rc             // O  return_code (0: no error)
) {

  // ************** //
  // 3.3 aggregator //  aggregate border outliers (L1), resistant outliers (L2) and untrusted top-outliers (L2)
  // ************** //
  // aggregate outliers in special "aggreg" clusters
  // invoked in post-clustering and in corrector after outlier correction and untrusted outlier correction
  // note: - aggregator starts from scratch (z.aggreg_size_2= aggreg_size_1)
  //       - at the end outliers will be sorted with respect to their values

  uint8_t  o_ind;         // index of current outlier
  uint8_t  o_last_ind;    // last index of outlier = z.outlier_size - 1
  uint16_t v_below;       // current outlier value
  uint16_t v_above;       // next higher outlier value
  int32_t  v_sum;         // total sum of all values in the subsequence
  uint8_t  v_count;       // number of values
  uint16_t center;        // aggregation mean value

  rc= CRC_0;
  z.aggreg_size_2= z.aggreg_size_1;
  if (z.outlier_size < 1) return;

  // _psln(F("v[z.outlier_ind[o_ind]] unsorted:"));
  // for (o_ind= 0; o_ind < z.outlier_size; o_ind++) {_pd(v[z.outlier_ind[o_ind]]);_ps("\t");}_psln("");

  // sort outliers with respect to the raw data values
  // =============
  // this is the only place where index_sort is used
  index_sort(v, z.outlier_ind, z.outlier_size);
/*PP
  _ps(F("outlier values  : "));
  for (o_ind= 0; o_ind < z.outlier_size; o_ind++) {_pd(v[z.outlier_ind[o_ind]]);_ps("\t");}_psln("");
*/

  o_last_ind= z.outlier_size - 1;
  o_ind= 0;
  do {
    if (z.aggreg_size_2 >= NC) {
      //E _psln(F("too many aggregations !!!)"));
      rc= CRC_4;
      return;
    }
    // _ps(F("START "));
    v_sum= 0;
    v_count= 0;
    do {
      v_below= v[z.outlier_ind[o_ind]];
      // _pd(v_below);_ps("\t");
      v_sum+= v_below;
      v_count++;
      if (o_ind >= o_last_ind) {
        // _ps(F(" ENDE X  "));
        center= v_sum / v_count;
        // _pd(center);_ps("\t");
        if (v_count > v_min_count) z.aggreg_center[z.aggreg_size_2++]= center & MSB;
        goto ENDE;
      }
      o_ind++;
      v_above= v[z.outlier_ind[o_ind]];
    } while ((v_below + (v_above >> 3)) > v_above);
    // above does not belong to the current aggregation
    // _ps(F(" ENDE Y  "));
    center= v_sum / v_count;
    // _pd(center);_ps("\t");
    // _psln("");
    if (v_count > v_min_count) z.aggreg_center[z.aggreg_size_2++]= center & MSB;
  } while (o_ind < o_last_ind);
  // last aggregation consists of a single value
  if (o_ind == o_last_ind)  {
    if (z.aggreg_size_2 >= NC) {
      //E _psln(F("too many aggregations !!!)"));
      rc= CRC_4;
      return;
    }
    // _ps(F("START "));
    // _pd(v[z.outlier_ind[o_ind]]);
    // _ps(F(" ENDE Z  "));
    v_count= 1;
    center= v[z.outlier_ind[o_ind]];
    // _pd(center);_ps("\t");
    if (v_count > v_min_count) z.aggreg_center[z.aggreg_size_2++]= center & MSB;
  } else {
    //E _psln(F("aggregator error (should never occur !!!)"));
    rc= CRC_17;
    return;
  }

ENDE:;
/*PP
  if (z.aggreg_size_2 > 0) _ps(F("outlier clusters: "));
  for (uint8_t a_ind= 0; a_ind < z.aggreg_size_2; a_ind++) {
    if (z.aggreg_center[a_ind] == 0) continue;
    _pd(z.aggreg_center[a_ind]);_ps("\t");
  }
  _psln("");_psln("");
*/
}
// END aggregator

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool classifier (      //     returns true, if a matching cluster or aggregation is found
  categories &z,       // I   current categories of either HIGH- or LOW- raw data values
  uint16_t  v_val,     // I   current value of which the matching or nearest cluster/aggregation should be found
  uint8_t  &cat_ind,   // O   resulting category index (cluster / aggregation)
  uint16_t &cat_val,   // O   resulting category value (center)
  uint8_t   option     // I   tightness: relative distance between value and category center (cluster/aggregation)
) {
  // ************** //
  // 3.4 classifier //  find the nearest category (comprising clusters and aggregations)
  // ************** //
  // given a raw data value v_val:
  // - return true  if - the nearest cluster is the matching category
  //                or - the cluster center is nearer than required (option)
  //                or - the aggreg  center is nearer than required (option)
  // - return false otherwise
  // note: aggregation indices start above the cluster indices

  // (A) try to find the cluster corresponding to v_val
  // --------------------------------------------------
  // success: return true in two cases
  //    - v_val belongs to cluster[cat_ind]
  //      z.cluster_floor[cat_ind] <= v_val < z.cluster_ceil[cat_ind]
  //    - v_val is near enough to a cluster center
  // failure: cat_ind and cat_val belong to the nearest cluster
  //    abs(v_val - z.cluster_center[cat_ind])= min!
  // note:
  // - clusters must be sorted in ascending order
  // - cluster size must be > 0
  // - if v_val is a HIGH value, then z must correspond to the HIGH-category!

  uint8_t  a_ind;   // index of aggregation
  uint16_t d1;      // distance to the above cluster (current)
  uint16_t d2;      // distance to the below cluster
  uint16_t delta;   // = abs( v_val - cat_val )

  cat_ind= 0;
  cat_val= 0;
  delta= CEIL_U;
  // start with the lowest cluster
  for (cat_ind= 0; cat_ind < z.cluster_size; cat_ind++) {
    if (v_val <  z.cluster_ceil[cat_ind]) goto HIGHER_CEIL_FOUND;
  }
  // v_val is higher than the highest cluster
  // set cat_ind to the nearest cluster, i.e. the highest one
  cat_ind= z.cluster_size - 1;
  delta=  v_val - z.cluster_center[cat_ind];
  goto CONTINUE_WITH_AGGREGATIONS;

HIGHER_CEIL_FOUND:
  // v_val is lower than the ceil of the current cluster

  // check floor of current cluster
  if (v_val >= z.cluster_floor[cat_ind]) {
    // v_val is higher or equal than the floor of the current cluster
    // cat_ind is the matching cluster!
    // ================================
    // delta= abs(z.cluster_center[cat_ind] - v_val);
    cat_val= z.cluster_center[cat_ind];
    return (true);
  }

  // v_val is lower than the floor of the current cluster
  if (cat_ind == 0) {
    // cat_ind == 0 is the nearest cluster, i.e. the lowest one
    delta=  z.cluster_center[cat_ind] - v_val;
    goto CONTINUE_WITH_AGGREGATIONS;
  }

  // v_val is between two clusters
  // cat_ind    : the cluster above v_val
  // cat_ind - 1: the cluster below v_val

  d1= z.cluster_center[cat_ind] - v_val;
  d2= v_val - z.cluster_center[cat_ind - 1];
  if (d1 < d2) {
    // the current cluster is the nearer one
    delta= d1;
  } else {
    // the cluster below v_val is the nearer one
    cat_ind-= 1;
    delta= d2;
  }

CONTINUE_WITH_AGGREGATIONS:
  // v_val does not match any cluster
  cat_val= z.cluster_center[cat_ind];

  // check whether cat_ind and cat_val can be attributed to a cluster (nearest cluster is near enough)
  if (delta < (cat_val >> option)) {
    // option= 4: the delta of the cluster is less than  6.25 %
    // option= 2: the delta of the cluster is less than 25.00 %
    return (true);
  }

  // (B) try to find the aggregation corresponding to the current value v_val
  // ------------------------------------------------------------------------
  // success: return true if
  //    - v_val is near enough to an aggregation_center:
  //      abs(aggreg_center[a_ind]- v_val) less than required by the option
  // failure: return false: in this case cat_ind and cat_val belong to the nearest aggregation
  // note   :
  //  - the start value of delta belongs to the nearest cluster <- (A) find cluster
  //  - the resulting aggregation indices are arranged above the cluster indices (i.e. increased by z.cluster_size)

  for (a_ind= 0; a_ind < z.aggreg_size_2; a_ind++) {
    d1= z.aggreg_center[a_ind];

    // absolute distance between value and center
    if (v_val > d1) d2= v_val - d1;
    else d2= d1 - v_val;

    if (d2 < delta) {
      cat_ind= z.cluster_size + a_ind;
      cat_val= d1;
      delta= d2;
    }
  }

  // the nearest category is a cluster or an aggregation
  if (delta < (cat_val >> option)) {
    // option= 4: the delta of the aggregation is less then 6.25%
    // option= 2: the delta of the aggregation is less then 22.5%
    return (true);
  }

  return (false);
}
// END classifier

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void sequence_printer (
  categories z[],    // I   categories of raw data values  ([1]: HIGH-duration_categories, [0]: LOW-duration_categories)
  uint16_t v[],      // I   flagged raw data value sequence: odd indices: HIGH-durations, even indices: LOW-durations
  int16_t  v_length  // I   number of signal durations
) {
  // ******************** //
  // 3.5 sequence_printer //  map the raw data into categorized sequence
  // ******************** //
  // mapping: continuous raw data -> discrete categories
  // print the categorized signal sequence, i.e. the duration-level sequence
  // first row: HIGH-, second row: LOW- indices of the corresponding category
  // special categories are marked as follows:
  // "!" : value belongs to an unreliable category
  // "*" : value is higher than the top-values barrier (-> subsequence separator like pause)
  // "-" : value is lower  than the floor of the lowest category (-> spike in front of a pause)
  // "?" : value does not belong to any category
  // print the correspondence between values and categories

  int16_t  j, k;
  uint8_t  z_ind;   // signal level index (either HIGH or LOW)
  int16_t  v_ind;   // index of signal sequence
  uint8_t  cat_ind; // index of current category (combined clusters and aggregations)
  uint16_t cat_val; // value of current category (combined clusters and aggregations)

  // end handling
  // ------------
  // the next two values after v_length are either (0, 0) or (x, CEIL)
  if ((v[v_length + 1] != 0) && (v[v_length + 2] != 0)) {
    v_length+= 2;
  }

  // print sequence index
  // --------------------
  //P _psln("");
  _ps(F("ind : "));
  k= 0;
  _pd(0);
  j= 2;
  for (v_ind= 0; v_ind <= v_length; v_ind+= 2) {
    if (j == 10) {
      if (++k == 10) k= 0;
      _pd(k);
      j= 2;
    } else {
      j+= 2;
      _ps(" ");
    }
  }
  _psln("");

  // print HIGH reliability marking
  // ------------------------------
  _ps(F("    : "));
  // HIGH: start at v[1]
  for (v_ind= (2-HIGH); v_ind <= v_length; v_ind+= 2) {
    // skip zero duration values (spike/drop)
    if (v[v_ind] == 0) {
      _ps(" ");
      continue;
    }
    if ((v[v_ind] & LSB) == RELIABLE) _ps(" ");
    else _ps(F("!"));
  }
  _psln("");

  // print HIGH / LOW categorized sequence
  // =====================================
  _ps("HIGH: "); // HIGH :
  for (z_ind= HIGH; ;z_ind= LOW) {

    for (v_ind= (2-z_ind); v_ind <= v_length; v_ind+= 2) {
      // skip zero duration values (spikes and drops)
      if (v[v_ind] == 0) {
        _ps(" ");
        continue;
      }
      // check whether the current value is above the barrier (includes CEIL!)
      if (v[v_ind] >= z[z_ind].separator_barrier) {
        _ps(F("*"));
        continue;
      }
      // check whether the current value is classifiable
      // !!! use the same C_OPT as in border values classification !!!
      if (classifier (z[z_ind], v[v_ind], cat_ind, cat_val, C_OPT_3)) {
        // the nearest category is near enough
        // the current value is classifiable
        // cat_ind contains the index of the category corresponding to v_val
        if (cat_ind < 10) _pd(cat_ind);
        // use characters for indices >= 10  ('a' = 97; 97 - 10 = 87)
        else _pc(87 + cat_ind);
        continue;
      }
      // the current value is not classifiable
      // check whether it is smaller than the smallest category
      if ((cat_ind == 0) && (v[v_ind] < cat_val)) _ps(F("-"));
      else _ps(F("?"));

    }
    _psln("");
    if (z_ind == LOW ) break;
    _ps("LOW : "); // LOW :
  }

  // print LOW reliability marking
  // -----------------------------
  _ps("    : ");
  // LOW: start at v[2]
  for (v_ind= (2-LOW); v_ind <= v_length; v_ind+= 2) {
    // skip zero duration values (spike/drop)
    if (v[v_ind] == 0) {
      _ps(" ");
      continue;
    }
    if ((v[v_ind] & LSB) == RELIABLE) _ps(" ");
    else _ps(F("!"));
  }
  _psln("");


  // print categories index
  // ----------------------
  _psln("");
  _psln(F("Categories"));
  //P _psln(F("----------"));
  _ps(F("ind : "));
  for (cat_ind= 0; cat_ind < max(z[HIGH].cluster_size + z[HIGH].aggreg_size_2, z[LOW].cluster_size + z[LOW].aggreg_size_2); cat_ind++) {
    _ps("\t");_pd(cat_ind);
  }
  _psln("");

  // print HIGH / LOW categories
  // ===========================
  _ps("HIGH: "); // HIGH :
  for (z_ind= HIGH; ;z_ind= LOW) {
    for (cat_ind= 0; cat_ind < z[z_ind].cluster_size; cat_ind++) {
      _ps("\t");_pd(z[z_ind].cluster_center[cat_ind]);
    }
    _ps(F(";"));
    for (cat_ind= 0; cat_ind < z[z_ind].aggreg_size_2; cat_ind++) {
      _ps("\t");_pd(z[z_ind].aggreg_center[cat_ind]);
    }

    _psln("");
    if (z_ind == LOW ) break;
    _ps("LOW : "); // LOW :
  }

}
// END sequence_printer

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

///*PP
void category_printer (
  categories &z,    // I  current categories of either HIGH- or LOW- raw data values
  uint16_t v[]      // I  flagged raw data value sequence: odd indices: HIGH-durations, even indices: LOW-durations
) {
  // ******************** //
  // 3.6 category_printer //  print the categories (clusters and aggregations)
  // ******************** //
  // print the categories (clusters and aggregations) of the signal durations
  int8_t  ind;      // index of cluster / aggregation

  _psln("");
  _psln(F("Clusters"));
  _psln(F("--------"));
  _ps(F("ind    count   floor  center   ceil  \n"));
  for (ind= 0; ind < z.cluster_size; ind++) {
    _pd(ind);        _ps("\t");
    _pd(z.cluster_count[ind]); _ps("\t");
    _pd(z.cluster_floor[ind]); _ps("\t");
    _pd(z.cluster_center[ind]);_ps("\t");
    _pd(z.cluster_ceil[ind]);  _ps("\t");
    _psln("");
  }
  _psln("");
  _ps(F("inlier count       : "));_ps("\t");_pdln(z.inlier_count);
  _ps(F("top-outlier barrier: "));_ps("\t");_pdln(z.separator_barrier);
  _ps(F("outlier size       : "));_ps("\t");_pdln(z.outlier_size);
  if (z.outlier_size > 0) {
    _ps(F("outlier indices    : "));_ps("\t");
    for (ind= 0; ind < z.outlier_size; ind++) {
      _pd(z.outlier_ind[ind]);_ps("\t");
    }
    _psln("");
    _ps(F("outlier values     : "));_ps("\t");
    for (ind= 0; ind < z.outlier_size; ind++) {
      _pd(v[z.outlier_ind[ind]]);_ps("\t");
    }
    _psln("");
  }
  if (z.aggreg_size_2 > 0) {
    _ps(F("aggregation centers: "));_ps("\t");
    for (ind= 0; ind < z.aggreg_size_2; ind++) {
      _pd(z.aggreg_center[ind]);_ps("\t");
    }
    _psln("");
  }
}   // end category_printer
//PP*/

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// ********** //
// 3.7 Helper //
// ********** //
// 3.7.1 sort: insertion sort (ascending)
// 3.7.2 index_sort: index insertion sort (ascending)
// 3.7.3 merge: merging of sorted arrays (without doubles)
// 3.7.4 statistics: compute mean, median and absolute deviation
// 3.7.5 miscellaneous
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void sort (
  uint16_t v[],   // IO  values to be sorted in ascending order
  uint16_t n      // I   number of elements in v
) {
  // ---------- //
  // 3.7.1 sort //  yaneurao's insertion sort  (ascending)
  // ---------- //

  uint16_t  i, j;
  uint16_t  tmp;
  for (i= 1; i < n; i++)
  {
    tmp= v[i];
    if (v[i-1] > tmp)
    {
      j= i;
      do {
      v[j]= v[j-1];
      --j;
      } while (j > 0 && v[j-1] > tmp);
      v[j]= tmp;
    }
  }
} // end sort

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void index_sort (
  uint16_t v[],       // I   indexed values (sort criteria)
  uint16_t v_ind[],   // IO  index of values to be sorted
  uint16_t n          // I   number of elements in v_ind
) {
  // ---------------- //
  // 3.7.2 index_sort //  yaneurao's index insertion sort  (ascending)
  // ---------------- //

  uint16_t  i, j;
  uint16_t  tmp_ind;
  for (i= 1; i < n; i++)
  {
    tmp_ind= v_ind[i];
    if (v[v_ind[i-1]] > v[tmp_ind])
    {
      j= i;
      do {
      v_ind[j]= v_ind[j-1];
      --j;
      } while (j > 0 && v[v_ind[j-1]] > v[tmp_ind]);
      v_ind[j]= tmp_ind;
    }
  }
} // end index_sort

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void merge (uint16_t a[], uint8_t na, uint16_t b[], uint8_t nb, uint16_t c[], uint8_t &nc)
{
  // ----------- //
  // 3.7.3 merge //  merging of sorted arrays (without doubles)
  // ----------- //
  // each array is sorted in ascending order without doubles
  // merge array a[i] with array b[j] into array c[k]

  int16_t i, j, k;
  i= j= k= 0;

  // nc must be <= NM
  nc= na + nb;

  // traverse both arrays
  while (i < na && j < nb) {
    if (a[i] < b[j]) {
      c[k++]=  a[i++];
    } else {
      c[k++]=  b[j++];
    }
  }

  // store remaining elements of a[]
  while (i < na) {
    c[k++]=  a[i++];
  }

  // store remaining elements of b[]
  while (j < nb) {
    c[k++]=  b[j++];
  }
} // end merge

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*
void statistics (
  categories &z,
  uint16_t v[],         // I  flagged raw data value sequence: odd indices: HIGH-durations, even indices: LOW-durations
  uint16_t v_start_ind, // I  start index of v[] (included)
  uint16_t v_stop_ind   // I  stop  index of v[] (included)
) {
  // ---------------- //
  // 3.7.4 statistics //  compute mean, median and absolute deviation
  // ---------------- //
  // raw data values
  uint16_t v_ind;     // index of current value
  uint16_t v_val;     // current value = v[v_ind]
  // current cluster
  uint8_t  c_ind;     // index of cluster
  uint16_t c_floor_val; // cluster floor (included)
  uint16_t c_ceil_val;  // cluster ceil  (not included)
  // statistics
  uint16_t w[NV+5];   // values in the range of the current cluster (between floor and ceil)
  uint16_t w_ind;     // index of value w[]
  uint16_t w_count;   // number of values in the range of the current cluster
  uint32_t w_sum;     // sum / sum of absolute deviations
  uint16_t mean;      // mean
  uint16_t med;       // median
  uint16_t mad;       // mean of absolute deviations

  bool filter= true;

  _psln("");
  _psln(F("mean, median and deviation"));
  _psln(F("c_ind   mean    med    mad"));
  for (c_ind= 0; c_ind < z.cluster_size; c_ind++) {
    _pd(c_ind);
    c_floor_val= z.cluster_floor[c_ind];
    c_ceil_val=  z.cluster_ceil[c_ind];
    // scan all elements of the sequence
    w_count= 0;
    for (v_ind= v_start_ind; v_ind <= v_stop_ind; v_ind+= 2) {
      // current value
      v_val= v[v_ind];
      // check bottom value range
      if (v_val <  c_floor_val) continue;
      // value filter
      if (filter) {
        // check immediate neighborhood for unreliable values
        // - element in front the current element
        if ((v[v_ind+1] & LSB) == UNRELIABLE) continue;
        // - current element
        if ((v[v_ind  ] & LSB) == UNRELIABLE) continue;
        // - element after the current element
        if ((v[v_ind-1] & LSB) == UNRELIABLE) continue;
      }
      // check top value range
      if (v_val >= c_ceil_val) {
        continue;
      }
      w[w_count]= v_val;
      w_count++;
    }
    // _pdln(w_count);
    if (w_count < 1) {
      _psln("");
      continue;
    }
    // sort
    sort(w, w_count);
    // for (w_ind= 0; w_ind < w_count; w_ind++) {
    //  _pd(w_ind);_ps("\t");_pdln(w[w_ind]);
    // }

    // mean
    w_sum= 0;
    for (w_ind= 0; w_ind < w_count; w_ind++) {
      w_sum+= w[w_ind];
    }
    mean= w_sum / w_count;
    _ps("\t");_pd(mean);

    // median
    if ((w_count & LSB) == 0) {
      med= (w[w_count/2 -1] + w[w_count/2]) / 2;
    } else {
      med= w[w_count/2];
    }
    _ps("\t");_pd(med);
    // mean absolute deviation
    w_sum= 0;
    for (w_ind= 0; w_ind < w_count; w_ind++) {
      w_sum+= abs(w[w_ind] - med);
    }
    mad= w_sum / w_count;
    _ps("\t");_pdln(mad);
  }
  _psln("");

} // end statistics
*/

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ------------------- //
// 3.7.5 miscellaneous //
// ------------------- //
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*
Histogram: bin-based computation of the median of a cluster
===========================================================
uint16_t bin_cluster_median (
  uint8_t  bin_count[],     // I  bin population of the current histogram
  uint16_t c_tot_sum,       // I  total population of the current cluster
  uint8_t  bin_start_ind,   // I  start_bin_index of the current cluster
  uint8_t  bin_stop_ind,    // I  stop_bin_index  of the current cluster
  uint8_t  bin_width_2log,  // I  2Log of bin_width of the current histogram
  uint16_t h_floor_val      // I  histogram base value : lowest value included in the current histogram
) {
  uint8_t  b_ind;       // index of bin : bin_count[b_ind]
  uint16_t bin_width;   // bin agglomeration width: value range per bin = 2 ** bin_width_2log
  uint16_t part_sum;    // partial frequency sum below the supposed median
  uint16_t diff_sum;    // difference between the partial frequency sum below and above the median
  uint16_t min_diff_sum;  // minimum difference between the partial frequency sum below and above the median
  uint16_t c_med;       // cluster median
  uint8_t  kk;          // 0: on bin / 1: between bins
  uint8_t  k2;          // bin count (step 2)
  uint8_t  k2_med;      // bin index of median (even: on bin; odd: between adjacent bins)

  // cluster median computed on the basis of the encompassed histogram bins
  bin_width= 1 << bin_width_2log;
  min_diff_sum= CEIL_U;
  k2_med= 0;
  for (kk= 0; kk < 2; kk++) {
    diff_sum= 0;
    part_sum= 0;
    k2= kk;
    for (b_ind= bin_start_ind; b_ind < bin_stop_ind - kk; b_ind++) {
      part_sum+= bin_count[b_ind-1+kk];
      // absolute delta between lower and upper sum (all unsigned!
      if (kk == 0) diff_sum= part_sum + part_sum + bin_count[b_ind];
      if (kk == 1) diff_sum= part_sum + part_sum;
      if (diff_sum > c_tot_sum)
         diff_sum= diff_sum - c_tot_sum;
      else diff_sum= c_tot_sum  - diff_sum;
      if (diff_sum < min_diff_sum) {
        min_diff_sum= diff_sum;
        k2_med= k2;
      }
      // _ps(F("diff_sum \t"));_pd(diff_sum);_ps(F("\t k2_med \t"));_pdln(k2_med);
      k2+= 2;
    }
  }
  c_med= (bin_start_ind << bin_width_2log) + (k2_med << (bin_width_2log - 1)) + h_floor_val + (bin_width >> 2);
//  c_med= (2 * bin_start_ind +  k2_med) * (bin_width / 2) + h_floor_val + (bin_width >> 2);
  // _ps(F("median "));_pd(k2_med);_ps(F(": \t"));_pdln(c_med);
  return (c_med);

} // end bin_cluster_median
*/

// ************************************************************************************************************

/*
Histogram: bin-based computation of the mean of a cluster
=========================================================
uint16_t bin_cluster_mean (
  uint8_t  bin_count[],     // I  bin frequency of the current histogram
  uint16_t c_tot_sum,       // I  total frequency sum of the current cluster
  uint8_t  bin_start_ind,   // I  start_bin_index of the current cluster
  uint8_t  bin_stop_ind,    // I  stop_bin_index  of the current cluster
  uint8_t  bin_width_2log,  // I  2Log of bin_width of the current histogram
  uint16_t h_floor_val      // I  histogram base value : lowest value included in the current histogram
) {
  uint16_t c_center_val;  // cluster mean
  uint8_t  b_ind;         // index of bin : bin_count[b_ind]
//  uint16_t bin_width;   // bin agglomeration width: value range per bin = 2 ** bin_width_2log
  uint32_t bin_sum; 
  uint32_t bin_mean;
  uint8_t  kk;

  // cluster bin mean
  kk= 1;
  bin_sum= 0;
  for (b_ind= bin_start_ind; b_ind < bin_stop_ind; b_ind++) {
    bin_sum+= kk * bin_count[b_ind];
    kk++;
  }
  bin_mean= (bin_sum << bin_width_2log) / c_tot_sum;
  _ps(F("bin_mean: "));_ps("\t");_pdln(bin_mean);

//  c_center_val= (bin_start_ind << bin_width_2log) + bin_mean + h_floor_val - (bin_width >> 1);
  c_center_val= (bin_start_ind << bin_width_2log) + bin_mean + h_floor_val - (1 << (bin_width_2log - 1));
  _ps(F("c_center_val: "));_ps("\t");_pdln(c_center_val);
  return (c_center_val);

} // end bin_cluster_mean
*/

// ************************************************************************************************************

/*
Fletcher16 Checksum
===================
An inefficient but straightforward implementation of a C language function to compute
the Fletcher-16 checksum of an array of 8-bit data elements follows:

uint16_t Fletcher16( uint8_t *data, int count )
{
   uint16_t sum1 = 0;
   uint16_t sum2 = 0;
   int index;

   for ( index = 0; index < count; ++index )
   {
      sum1 = (sum1 + data[index]) % 255;
      sum2 = (sum2 + sum1) % 255;
   }
   return (sum2 << 8) | sum1;
}
*/

// ************************************************************************************************************
