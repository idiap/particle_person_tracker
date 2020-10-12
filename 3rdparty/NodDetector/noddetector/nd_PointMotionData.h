/**
 * @file noddetector/nd_PointMotionData.h
 * @date 04 March 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 * @author Laurent Nguyen <lnguyen@idiap.ch>
 *
 * @brief Representation of point motion data
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __ND_POINTMOTIONDATA_H__
#define __ND_POINTMOTIONDATA_H__

// SYSTEM INCLUDES
#include <vector>                                  // STL vector

namespace NodDetector {

/// @brief Class to represent point motion data temporal series.
///
/// This class represents point motion data temporal series consisting of two
/// vectors of floating point numbers - X and Y motion vector coordinates
/// of a point.
///
/// @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
/// @author Laurent Nguyen <lnguyen@idiap.ch>
/// @version 1.0
/// @date    04.03.2013

struct nd_PointMotionData {

    // TYPES

    /// Floating point type used to describe point motion data.
    typedef double RealType;

    // LIFECYCLE

    /// Constructs point motion data vectors of certain length
    /// @param size Length of constructed point motion data vectors
    explicit nd_PointMotionData(size_t size) : m_XMotionData(size, 0),
        m_YMotionData(size, 0) {
    } // nd_PointMotionData

    /// Constructs a copy of point motion data vectors
    /// @param other Point motion data vectors to copy
    nd_PointMotionData(const nd_PointMotionData& other) :
        m_XMotionData(other.m_XMotionData),
        m_YMotionData(other.m_YMotionData) {
    } // nd_PointMotionData

    /// Destructor
    ~nd_PointMotionData() {
    } // ~nd_PointMotionData

    // OPERATIONS

    /// Assigns contents of other point motion data structure to this
    /// @param other Point motion data structure to assign
    /// @return Reference to this point motion data structure
    nd_PointMotionData& operator=(const nd_PointMotionData& other) {
        if (this != &other) {
            m_XMotionData = other.m_XMotionData;
            m_YMotionData = other.m_YMotionData;
        }
        return *this;
    } // operator=

    // FIELDS

    /// Point motion data along X axis
    std::vector<RealType> m_XMotionData;
    /// Point motion data along Y axis
    std::vector<RealType> m_YMotionData;

}; // struct nd_PointMotionData

/// Stream output operator for point motion data
std::ostream& operator<<(std::ostream& out,
        const nd_PointMotionData& data);

} // namespace NodDetector

#endif // __ND_POINTMOTIONDATA_H__
