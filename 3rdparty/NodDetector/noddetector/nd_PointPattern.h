/**
 * @file noddetector/nd_PointPattern.h
 * @date 04 March 2013
 * @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
 * @author Laurent Nguyen <lnguyen@idiap.ch>
 *
 * @brief Representation of a point pattern
 *
 * Copyright (c) 2011-2020 Idiap Research Institute, Martigny, Switzerland
 *
 * See COPYING file for the complete license text.
 *
 */

#ifndef __ND_POINTPATTERN_H__
#define __ND_POINTPATTERN_H__

// SYSTEM INCLUDES
#include <vector>                                  // STL vector
#include <cassert>                                 // assertions
#include <boost/foreach.hpp>                       // boost FOREACH loop

namespace NodDetector {

/// @brief Struct to represent a 2D point.
///
/// Point is a 2D vector space element with X and Y coordinates.
///
/// @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
/// @author Laurent Nguyen <lnguyen@idiap.ch>
/// @version 1.0
/// @date   04.03.2013

struct nd_Point {

    // TYPES

    /// Floating point type used to describe point coordinates.
    typedef double RealType;

    // FIELDS

    /// X point coordinate
    RealType m_X;
    /// Y point coordinate
    RealType m_Y;
};

/// @brief Class to represent a point pattern.
///
/// Point pattern is a collection of points with coordinates given relative
/// to some bounding box. Thus point coordinates should be non-negative and
/// not greater than 1 (0 <= x <= 1).
///
/// @author Vasil Khalidov <Vasil.Khalidov@idiap.ch>
/// @author Laurent Nguyen <lnguyen@idiap.ch>
/// @version 1.0
/// @date   04.03.2013

struct nd_PointPattern {

    // LIFECYCLE

    /// Constructs point pattern from a vector of relative point coordinates.
    /// Point coordinates are given with respect to some bounding box
    /// and should be non-negative and not greater than 1 (0 <= x <= 1).
    /// @param points Vector of points in a pattern
    nd_PointPattern(const std::vector<nd_Point>& points) : m_Points(points) {
        BOOST_FOREACH(const nd_Point& point, points) {
            assert((point.m_X >= 0) && (point.m_X <= 1));
            assert((point.m_Y >= 0) && (point.m_Y <= 1));
        }
    } // nd_PointPattern

    /// Constructs a copy of a point pattern
    /// @param other Point pattern to copy
    nd_PointPattern(const nd_PointPattern& other) : m_Points(other.m_Points) {
    } // nd_PointPattern

    /// Destructor
    ~nd_PointPattern() {
    } // ~nd_PointPattern

    // OPERATIONS

    /// Assignment operator for point patterns
    /// @param other Point pattern to assign
    /// @return Reference to this point pattern structure
    nd_PointPattern& operator=(const nd_PointPattern& other) {
        if (this != &other) {
            m_Points = other.m_Points;
        }
        return *this;
    } // operator=

    // FIELDS

    /// Points locations (relative to some bounding box)
    /// for every point in a pattern
    std::vector<nd_Point> m_Points;

}; // struct nd_PointPattern

} // namespace NodDetector

#endif // __ND_POINTPATTERN_H__
