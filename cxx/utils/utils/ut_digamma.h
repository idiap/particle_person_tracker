/**
 * @file cxx/utils/utils/ut_digamma.h
 * @date 03 February 2011
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 *
 * @brief Approximation to digamma function computation
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __UT_DIGAMMA__
#define __UT_DIGAMMA__

#include <cassert>
#include <cmath>

namespace TTrackUtils {

/// Approximation to digamma function, based on series expansion of
/// digamma(x) - log(x).
template <typename T>
inline T fast_digamma(T x) {
//    if (x <= 0) {
//        cout << "digamma function: x = " << x << " <= 0" << endl;
//    }
  assert(x > 0);
  float result = 0;
  float xx, xx2;
  while(x < 7) {
    // The approximation is valid for x >> 1 only, so use the identity
    // [digamma(x) = digamma(x+1) - 1/x] to get x in right range.
      result -= 1/x;
      x++;
  }
  xx = 1.0f / (x - 0.5);
  xx2 = xx * xx;
  result +=
      log(x) + xx2 * (
      (1./24.) - xx2 * (
        (7./960.) + xx2 * (
          (31./8064.) - xx2*(127./30720.))));
  return result;
}

} // namespace TTrackUtils

#endif // __UT_DIGAMMA__
