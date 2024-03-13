/**
 * \file Constants.h
 * \brief Header for GeographicLib::Constants class
 *
 * Copyright (c) Charles Karney (2008-2022) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_CONSTANTS_HPP)
#define GEOGRAPHICLIB_CONSTANTS_HPP 1

#include <AP_Geoid/Config.h>
#include <AP_Geoid/Math.h>

/**
 * \brief Namespace for %GeographicLib
 *
 * All of %GeographicLib is defined within the GeographicLib namespace.  In
 * addition all the header files are included via %GeographicLib/Class.h.
 * This minimizes the likelihood of conflicts with other packages.
 **********************************************************************/
namespace GeographicLib {

  /**
   * \brief %Constants needed by %GeographicLib
   *
   * Define constants specifying the WGS84 ellipsoid, the UTM and UPS
   * projections, and various unit conversions.
   *
   * Example of use:
   * \include example-Constants.cpp
   **********************************************************************/
  class Constants {
  private:
    typedef Math::real real;
    Constants() = delete;       // Disable constructor

  public:
    /** \name Ellipsoid parameters
     **********************************************************************/
    ///@{
    /**
     * @tparam T the type of the returned value.
     * @return the equatorial radius of WGS84 ellipsoid (6378137 m).
     **********************************************************************/
    template<typename T = real> static T WGS84_a()
    { return 6378137 * meter<T>(); }
    /**
     * @tparam T the type of the returned value.
     * @return the flattening of WGS84 ellipsoid (1/298.257223563).
     **********************************************************************/
    template<typename T = real> static T WGS84_f() {
      // Evaluating this as 1000000000 / T(298257223563LL) reduces the
      // round-off error by about 10%.  However, expressing the flattening as
      // 1/298.257223563 is well ingrained.
      return 1 / ( T(298257223563LL) / 1000000000 );
    }
    ///@}

    /** \name SI units
     **********************************************************************/
    ///@{
    /**
     * @tparam T the type of the returned value.
     * @return the number of meters in a meter.
     *
     * This is unity, but this lets the internal system of units be changed if
     * necessary.
     **********************************************************************/
    template<typename T = real> static T meter() { return T(1); }
    ///@}
  };
} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_CONSTANTS_HPP
