/**
 * @file noddetector/nd_Utils.h
 * @date 14 March 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 * @author Laurent Nguyen <lnguyen@idiap.ch>
 *
 * @brief Utility methods for nod detector
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __ND_UTILS_H__
#define __ND_UTILS_H__

// SYSTEM INCLUDES
#include <vector>

namespace NodDetector {

enum nd_InterpolationType {
    ND_INTERP_LINEAR = 0,
    ND_INTERP_POLYNOMIAL = 1,
    ND_INTERP_CSPLINE = 2
};

/// Interpolates function values observed at given timestamps to new timestamps
/// using provided interpolation type
/// @param ts Timestamps of measurements
/// @param values Measured function values, should be of the same size as
///        timestamps
/// @param ts_new New timestamps for which to evaluate the function
/// @param values_new Output interpolated function values, should be of the
///        same size as timestamps_new
/// @param interpolation_type Interpolation type to be used
void interpolate(const std::vector<double>& ts,
        const std::vector<double>& values,
        const std::vector<double>& ts_new,
        std::vector<double>& values_new,
        nd_InterpolationType interpolation_type = ND_INTERP_LINEAR);

/// Interpolates motion vector values observed at given timestamps
/// to new timestamps using interpolation of given type.
/// Note that to interpolate motion vectors we take a point at 0 and
/// accumulate displacements using motion values to produce
/// point coordinates at given timestamps. Then we interpolate the trajectory
/// using point coordinates and new timestamps. Finally, we derive new motion
/// vectors that correspond to new timestamps.
/// @param prev_ts Timestamp of start of measurement of the first motion vector
/// @param ts Timestamps of measurements
/// @param values Measured motion values, should be of the same size as
///        timestamps
/// @param ts_new New timestamps for which to evaluate the function
/// @param values_new Output interpolated motion values, should be of the
///        same size as timestamps_new
/// @param interpolation_type Interpolation type to be used
void interpolate_motion(double prev_ts,
        const std::vector<double>& ts,
        const std::vector<double>& values,
        const std::vector<double>& ts_new,
        std::vector<double>& values_new,
        nd_InterpolationType interpolation_type = ND_INTERP_LINEAR);

} // namespace NodDetector

#endif // __ND_UTILS_H__
