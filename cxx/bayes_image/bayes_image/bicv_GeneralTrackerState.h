// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_GeneralTrackerState - class for a general particle filter state for
//                            an object tracker
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

#ifndef __BICV_GENERALTRACKERSTATE_H__
#define __BICV_GENERALTRACKERSTATE_H__

// SYSTEM INCLUDES
#include <iostream>       // standard io streams library
#include <boost/date_time/posix_time/posix_time.hpp>       // boost posix time

namespace BICV {

/// @brief Class to represent tracker parameters
///
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    24.10.2011

struct bicv_TrackerParameters {
    // bounding box parameters
    float m_TranslationX;
    float m_TranslationY;
    float m_Scale;
    float m_Excentricity;

    // pose index
    unsigned m_PoseIndex;

    /// Current time associated with the parameters, used when working with
    /// random processes sampled at irregularly spaced times
    boost::posix_time::ptime m_Time;

    bicv_TrackerParameters& operator+=(
            const bicv_TrackerParameters& rhs) {
        m_TranslationX += rhs.m_TranslationX;
        m_TranslationY += rhs.m_TranslationY;
        m_Scale += rhs.m_Scale;
        m_Excentricity += rhs.m_Excentricity;
        return *this;
    }
    bicv_TrackerParameters& operator-=(
            const bicv_TrackerParameters& rhs) {
        m_TranslationX -= rhs.m_TranslationX;
        m_TranslationY -= rhs.m_TranslationY;
        m_Scale -= rhs.m_Scale;
        m_Excentricity -= rhs.m_Excentricity;
        return *this;
    }
    bicv_TrackerParameters& operator*=(
            const bicv_TrackerParameters& rhs) {
        m_TranslationX *= rhs.m_TranslationX;
        m_TranslationY *= rhs.m_TranslationY;
        m_Scale *= rhs.m_Scale;
        m_Excentricity *= rhs.m_Excentricity;
        return *this;
    }
    bicv_TrackerParameters& operator*=(float val) {
        m_TranslationX *= val;
        m_TranslationY *= val;
        m_Scale *= val;
        m_Excentricity *= val;
        return *this;
    }
    bicv_TrackerParameters& operator/=(
            const bicv_TrackerParameters& rhs) {
        m_TranslationX *= rhs.m_TranslationX;
        m_TranslationY *= rhs.m_TranslationY;
        m_Scale *= rhs.m_Scale;;
        m_Excentricity *= rhs.m_Excentricity;
        return *this;
    }
    bicv_TrackerParameters& operator/=(
            float val) {
        m_TranslationX /= val;
        m_TranslationY /= val;
        m_Scale /= val;
        m_Excentricity /= val;
        return *this;
    }
};

/// @brief Class to represent tracker state
///
/// This class represents a general tracker state consisting of current and
/// previous parameter values.
/// @author  Vasil Khalidov (vasil.khalidov@idiap.ch)
/// @version 2.0
/// @date    15.02.2011

struct bicv_TrackerState {
    // current head pose parameters
    bicv_TrackerParameters m_ParamsCur;
    // previous head pose parameters
    bicv_TrackerParameters m_ParamsPrev;
    // particle label, reserved for internal use
    int m_Label;

    bicv_TrackerState& operator+=(
            const bicv_TrackerState& rhs) {
        m_ParamsCur += rhs.m_ParamsCur;
        m_ParamsPrev += rhs.m_ParamsPrev;
        return *this;
    }
    bicv_TrackerState& operator-=(
            const bicv_TrackerState& rhs) {
        m_ParamsCur -= rhs.m_ParamsCur;
        m_ParamsPrev -= rhs.m_ParamsPrev;
        return *this;
    }
    bicv_TrackerState& operator*=(
            const bicv_TrackerState& rhs) {
        m_ParamsCur *= rhs.m_ParamsCur;
        m_ParamsPrev *= rhs.m_ParamsPrev;
        return *this;
    }
    bicv_TrackerState& operator/=(
            const bicv_TrackerState& rhs) {
        m_ParamsCur /= rhs.m_ParamsCur;
        m_ParamsPrev /= rhs.m_ParamsPrev;
        return *this;
    }
};

float distance_scaled(const BICV::bicv_TrackerParameters& hp1,
        const BICV::bicv_TrackerParameters& hp2);

} // namespace BICV

/// Output operator.
/// @param os Output stream
/// @param hp Head pose parameters structure
/// @return Output stream
std::ostream& operator<<( std::ostream& os,
        const BICV::bicv_TrackerParameters& hp);

/// Addition operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise sum of the two head pose parameters structures.
BICV::bicv_TrackerParameters operator+(
    const BICV::bicv_TrackerParameters& hp1,
    const BICV::bicv_TrackerParameters& hp2);

/// Substraction operator.
/// @param s1 Augend parameters structure
/// @param s2 Addend parameters structure
/// @return The elementwise difference of the two parameters structures.
BICV::bicv_TrackerParameters operator-(
    const BICV::bicv_TrackerParameters& s1,
    const BICV::bicv_TrackerParameters& s2);

/// Elementwise multiplication operator.
///
/// @param hp1 Multiplicand head pose parameters structure
/// @param hp2 Multiplier head pose parameters structure
/// @return The elementwise product of the two head pose parameters structures.
BICV::bicv_TrackerParameters operator*(
    const BICV::bicv_TrackerParameters& hp1,
    const BICV::bicv_TrackerParameters& hp2);

/// Elementwise multiplication operator.
///
/// @param hp Multiplicand head pose parameters structure
/// @param val Multiplier argument
/// @return The elementwise product of the head pose parameters structure.
BICV::bicv_TrackerParameters operator*(const BICV::bicv_TrackerParameters& hp,
        float val);

/// Elementwise multiplication operator.
///
/// @param hp Multiplicand head pose parameters structure
/// @param val Multiplier argument
/// @return The elementwise product of the head pose parameters structure.
BICV::bicv_TrackerParameters operator*(float val,
        const BICV::bicv_TrackerParameters& hp);

/// Output operator.
/// @param os Output stream
/// @param hp Head pose parameters structure
/// @return Output stream
std::ostream& operator<<( std::ostream& os,
        const BICV::bicv_TrackerState& hp);

/// Addition operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise sum of the two head pose parameters structures.
BICV::bicv_TrackerState operator+(
    const BICV::bicv_TrackerState& hp1,
    const BICV::bicv_TrackerState& hp2);

/// Substraction operator.
/// @param s1 Augend state
/// @param s2 Addend state
/// @return The elementwise difference of the two states.
BICV::bicv_TrackerState operator-(
    const BICV::bicv_TrackerState& s1,
    const BICV::bicv_TrackerState& s2);

/// Multiplication operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise sum of the two head pose parameters structures.
BICV::bicv_TrackerState operator*(
    const BICV::bicv_TrackerState& hp1,
    const BICV::bicv_TrackerState& hp2);

/// Multiplication operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise sum of the two head pose parameters structures.
BICV::bicv_TrackerState operator*(
    const BICV::bicv_TrackerState& hp1,
    float hp2);

/// Multiplication operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise sum of the two head pose parameters structures.
BICV::bicv_TrackerState operator*(float hp2,
    const BICV::bicv_TrackerState& hp1);

/// Multiplication operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise sum of the two head pose parameters structures.
BICV::bicv_TrackerState operator/(
    const BICV::bicv_TrackerState& hp1,
    float hp2);

#endif // __BICV_GENERALTRACKERSTATE_H__
