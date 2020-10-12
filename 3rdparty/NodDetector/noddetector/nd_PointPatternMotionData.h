/**
 * @file noddetector/nd_PointPatternMotionData.h
 * @date 04 March 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 * @author Laurent Nguyen <lnguyen@idiap.ch>
 *
 * @brief Representation of point motion data for a point pattern
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __ND_POINTPATTERNMOTIONDATA_H__
#define __ND_POINTPATTERNMOTIONDATA_H__

// LOCAL INCLUDES
#include "nd_PointMotionData.h"                           // point motion

namespace NodDetector {

/// @brief Class to represent point motion data temporal series for a pattern
/// of points.
///
/// This class represents point motion data temporal series for a pattern
/// of points. For every point in a pattern data consisting of two
/// vectors of floating point numbers is kept - X and Y motion vector
/// coordinates of a point.
///
/// @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
/// @author Laurent Nguyen <lnguyen@idiap.ch>
/// @version 1.0
/// @date   04.03.2013

struct nd_PointPatternMotionData {

    // LIFECYCLE

    /// Constructs point pattern motion data vectors of certain length.
    /// @param pattern_size Length of a point pattern
    /// @param motion_data_size Length of point motion data vectors
    nd_PointPatternMotionData(size_t pattern_size, size_t motion_data_size) {
        nd_PointMotionData motion_data_example(motion_data_size);
        m_PointMotionDatas.resize(pattern_size, motion_data_example);
    } // nd_PointMotionPattern

    /// Constructs a copy of point pattern motion data
    /// @param other Point pattern motion data to copy
    nd_PointPatternMotionData(const nd_PointPatternMotionData& other) :
        m_PointMotionDatas(other.m_PointMotionDatas) {
    } // nd_PointMotionPattern


    /// Destructor
    ~nd_PointPatternMotionData() {
    } // ~nd_PointMotionPattern


    // OPERATIONS

    /// Assigns contents of other point pattern motion data structure to this
    /// @param other Point pattern motion data structure to assign
    /// @return Reference to this point pattern motion data structure
    nd_PointPatternMotionData& operator=(const nd_PointPatternMotionData& other) {
        if (this != &other) {
            m_PointMotionDatas = other.m_PointMotionDatas;
        }
        return *this;
    } // operator=

    // FIELDS

    /// Point motion data structure for every point in a pattern
    std::vector<nd_PointMotionData> m_PointMotionDatas;

}; // struct nd_PointPatternMotionData

/// Stream output operator for pattern motion data
std::ostream& operator<<(std::ostream& out,
        const nd_PointPatternMotionData& data);

} // namespace NodDetector

#endif // __ND_POINTPATTERNMOTIONDATA_H__
