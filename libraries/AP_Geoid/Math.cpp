/**
 * \file Math.cpp
 * \brief Implementation for GeographicLib::Math class
 *
 * Copyright (c) Charles Karney (2015-2022) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <AP_Geoid/Math.h>
#include <AP_Math/AP_Math.h>

namespace GeographicLib {

  using namespace std;

  template<typename T> T Math::sum(T u, T v, T& t) {
    GEOGRAPHICLIB_VOLATILE T s = u + v;
    GEOGRAPHICLIB_VOLATILE T up = s - v;
    GEOGRAPHICLIB_VOLATILE T vpp = s - up;
    up -= u;
    vpp -= v;
    // if s = 0, then t = 0 and give t the same sign as s
    // mpreal needs T(0) here
    t = !is_zero(s) ? T(0) - (up + vpp) : s;
    // t = s != 0 ? T(0) - (up + vpp) : s;
    // u + v =       s      + t
    //       = round(u + v) + t
    return s;
  }

  template<typename T> T Math::NaN() {
    return numeric_limits<T>::has_quiet_NaN ?
      numeric_limits<T>::quiet_NaN() :
      numeric_limits<T>::max();
  }

  // Instantiate
#define GEOGRAPHICLIB_MATH_INSTANTIATE(T)           \
  template T    Math::sum          <T>(T, T, T&);   \
  template T    Math::AngNormalize <T>(T);          \
  template T    Math::NaN          <T>();

  GEOGRAPHICLIB_MATH_INSTANTIATE(ftype)

#undef GEOGRAPHICLIB_MATH_INSTANTIATE

} // namespace GeographicLib
