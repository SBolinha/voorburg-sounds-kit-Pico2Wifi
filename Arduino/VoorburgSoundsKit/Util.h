/**

 Some utility functions
  
*/

#ifndef _UTIL_H_
#define _UTIL_H_

// Calculate the frequency of a bin depending on sameplerate and FFT size.
#define FFT_BIN(num, fs, bins) (num*((float)fs/(float)bins))

// RMS to decibel
const double sqrt2 = sqrt(2.0);
float db_full_scale_amp(double rms) {
  return (20 * log10(rms * sqrt2));
}
float db_full_scale_pow(double rms) {
  return (10 * log10(rms * 2.0f));
}

// Qmath adapted from here: https://www.dsprelated.com/showcode/40.php
// More info here: http://www.hugi.scene.org/online/coding/hugi%2015%20-%20cmtadfix.htm
typedef int64_t qlong;
#define double2q(x, fb) ((qlong)((x) * ((qlong)1 << (fb))))
#define q2double(x, fb) ((double)(x) / ((qlong)1 << (fb)))

#endif
