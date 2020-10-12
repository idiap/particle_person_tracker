// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_HeadPoseTrackerState - class for a head pose particle filter state
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __BICV_HEADPOSETRACKERSTATE_H__
#define __BICV_HEADPOSETRACKERSTATE_H__

// SYSTEM INCLUDES
#include "boost/date_time/posix_time/posix_time.hpp"       // boost posix time

// PROJECT INCLUDES
#include <opencvplus/cvp_HeadPose.h>                       // head pose

namespace BICV {

/// @brief Class to represent head pose parameters
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    15.02.2011

struct bicv_HeadPoseParameters {
    /// Bounding box X coordinate translation (X coordinate of the middle of
    /// bounding box in pixels)
    float m_TranslationX;
    /// Bounding box Y coordinate translation (Y coordinate of the middle of
    /// bounding box in pixels)
    float m_TranslationY;
    /// Bounding box scale (bounding box width divided by 200)
    float m_Scale;
    /// Bounding box excentricity (ratio of bounding box height over its width)
    float m_Excentricity;
    /// Head pose parameters (pan, tilt, roll)
    OpenCvPlus::cvp_HeadPose<> m_HeadPose;
    /// Current time associated with the parameters, used when working with
    /// random processes sampled at irregularly spaced times
    boost::posix_time::ptime m_Time;


    bicv_HeadPoseParameters& operator+=(
            const bicv_HeadPoseParameters& rhs) {
        m_TranslationX += rhs.m_TranslationX;
        m_TranslationY += rhs.m_TranslationY;
        m_Scale += rhs.m_Scale;
        m_Excentricity += rhs.m_Excentricity;
        m_HeadPose += rhs.m_HeadPose;
        return *this;
    }
    bicv_HeadPoseParameters& operator-=(
            const bicv_HeadPoseParameters& rhs) {
        m_TranslationX -= rhs.m_TranslationX;
        m_TranslationY -= rhs.m_TranslationY;
        m_Scale -= rhs.m_Scale;
        m_Excentricity -= rhs.m_Excentricity;
        m_HeadPose -= rhs.m_HeadPose;
        return *this;
    }
    bicv_HeadPoseParameters& operator*=(
            const bicv_HeadPoseParameters& rhs) {
        m_TranslationX *= rhs.m_TranslationX;
        m_TranslationY *= rhs.m_TranslationY;
        m_Scale *= rhs.m_Scale;
        m_Excentricity *= rhs.m_Excentricity;
        m_HeadPose *= rhs.m_HeadPose;
        return *this;
    }
    bicv_HeadPoseParameters& operator*=(float val) {
        m_TranslationX *= val;
        m_TranslationY *= val;
        m_Scale *= val;
        m_Excentricity *= val;
        m_HeadPose *= val;
        return *this;
    }
    bicv_HeadPoseParameters& operator/=(
            const bicv_HeadPoseParameters& rhs) {
        m_TranslationX *= rhs.m_TranslationX;
        m_TranslationY *= rhs.m_TranslationY;
        m_Scale *= rhs.m_Scale;;
        m_Excentricity *= rhs.m_Excentricity;
        m_HeadPose *= rhs.m_HeadPose;
        return *this;
    }
    bicv_HeadPoseParameters& operator/=(
            float val) {
        m_TranslationX /= val;
        m_TranslationY /= val;
        m_Scale /= val;
        m_Excentricity /= val;
        m_HeadPose /= val;
        return *this;
    }
};

/// @brief Class to represent head pose tracker state
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    15.02.2011

struct bicv_HeadPoseARTrackerState {
    /// Current head pose parameters
    bicv_HeadPoseParameters m_HeadPoseParamsCur;
    /// Previous head pose parameters
    bicv_HeadPoseParameters m_HeadPoseParamsPrev;
    /// particle label, reserved for internal use
    int m_Label;

    bicv_HeadPoseARTrackerState& operator+=(
            const bicv_HeadPoseARTrackerState& rhs) {
        m_HeadPoseParamsCur += rhs.m_HeadPoseParamsCur;
        m_HeadPoseParamsPrev += rhs.m_HeadPoseParamsPrev;
        return *this;
    }
    bicv_HeadPoseARTrackerState& operator-=(
            const bicv_HeadPoseARTrackerState& rhs) {
        m_HeadPoseParamsCur -= rhs.m_HeadPoseParamsCur;
        m_HeadPoseParamsPrev -= rhs.m_HeadPoseParamsPrev;
        return *this;
    }
    bicv_HeadPoseARTrackerState& operator*=(
            const bicv_HeadPoseARTrackerState& rhs) {
        m_HeadPoseParamsCur *= rhs.m_HeadPoseParamsCur;
        m_HeadPoseParamsPrev *= rhs.m_HeadPoseParamsPrev;
        return *this;
    }
    bicv_HeadPoseARTrackerState& operator/=(
            const bicv_HeadPoseARTrackerState& rhs) {
        m_HeadPoseParamsCur /= rhs.m_HeadPoseParamsCur;
        m_HeadPoseParamsPrev /= rhs.m_HeadPoseParamsPrev;
        return *this;
    }
};

float distance_scaled(const BICV::bicv_HeadPoseParameters& hp1,
        const BICV::bicv_HeadPoseParameters& hp2);

bool intersects(const BICV::bicv_HeadPoseParameters& hp1,
        const BICV::bicv_HeadPoseParameters& hp2);

} // namespace BICV

/// Output operator.
/// @param os Output stream
/// @param hp Head pose parameters structure
/// @return Output stream
std::ostream& operator<<( std::ostream& os,
        const BICV::bicv_HeadPoseParameters& hp);

/// Addition operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise sum of the two head pose parameters structures.
BICV::bicv_HeadPoseParameters operator+(
    const BICV::bicv_HeadPoseParameters& hp1,
    const BICV::bicv_HeadPoseParameters& hp2);

/// Subtraction operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise difference (hp1-hp2) of the two head pose parameters structures.
BICV::bicv_HeadPoseParameters operator-(
    const BICV::bicv_HeadPoseParameters& hp1,
    const BICV::bicv_HeadPoseParameters& hp2);

/// Elementwise multiplication operator.
/// @param hp1 Multiplicand head pose parameters structure
/// @param hp2 Multiplier head pose parameters structure
/// @return The elementwise product of the two head pose parameters structures.
BICV::bicv_HeadPoseParameters operator*(
    const BICV::bicv_HeadPoseParameters& hp1,
    const BICV::bicv_HeadPoseParameters& hp2);

/// Elementwise multiplication operator.
///
/// @param hp Multiplicand head pose parameters structure
/// @param val Multiplier argument
/// @return The elementwise product of the head pose parameters structure.
BICV::bicv_HeadPoseParameters operator*(const BICV::bicv_HeadPoseParameters& hp,
        float val);

/// Elementwise multiplication operator.
///
/// @param hp Multiplicand head pose parameters structure
/// @param val Multiplier argument
/// @return The elementwise product of the head pose parameters structure.
BICV::bicv_HeadPoseParameters operator*(float val,
        const BICV::bicv_HeadPoseParameters& hp);

/// Output operator.
/// @param os Output stream
/// @param hp Head pose parameters structure
/// @return Output stream
std::ostream& operator<<( std::ostream& os,
        const BICV::bicv_HeadPoseARTrackerState& hp);

/// Addition operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise sum of the two head pose parameters structures.
BICV::bicv_HeadPoseARTrackerState operator+(
    const BICV::bicv_HeadPoseARTrackerState& hp1,
    const BICV::bicv_HeadPoseARTrackerState& hp2);

/// Subtraction operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise difference (hp1-hp2) of the two head pose parameters structures.
BICV::bicv_HeadPoseARTrackerState operator-(
    const BICV::bicv_HeadPoseARTrackerState& hp1,
    const BICV::bicv_HeadPoseARTrackerState& hp2);

/// Multiplication operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise sum of the two head pose parameters structures.
BICV::bicv_HeadPoseARTrackerState operator*(
    const BICV::bicv_HeadPoseARTrackerState& hp1,
    const BICV::bicv_HeadPoseARTrackerState& hp2);

/// Multiplication operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise sum of the two head pose parameters structures.
BICV::bicv_HeadPoseARTrackerState operator*(
    const BICV::bicv_HeadPoseARTrackerState& hp1,
    float hp2);

/// Multiplication operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise sum of the two head pose parameters structures.
BICV::bicv_HeadPoseARTrackerState operator*(float hp2,
    const BICV::bicv_HeadPoseARTrackerState& hp1);

/// Multiplication operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise sum of the two head pose parameters structures.
BICV::bicv_HeadPoseARTrackerState operator/(
    const BICV::bicv_HeadPoseARTrackerState& hp1,
    float hp2);

#endif // __BICV_HEADPOSETRACKERSTATE_H__
