// Copyright (c) 2010-2020 Idiap Research Institute
//
// bicv_HeadPoseTrackerState - class for a head pose particle filter state
//
// Authors: Vasil Khalidov (vasil.khalidov@idiap.ch)
//
// See COPYING file for the complete license text.

// SYSTEM INCLUDES
#include <cmath>                                         // std math functions

// LOCAL INCLUDES
#include <bayes_image/bicv_HeadPoseTrackerState.h>       // declaration of this

namespace BICV {
float distance_scaled(const BICV::bicv_HeadPoseParameters& hp1,
        const BICV::bicv_HeadPoseParameters& hp2) {
    float dist_x = fabs(hp1.m_TranslationX - hp2.m_TranslationX) *
            (0.5f / hp1.m_Scale + 0.5f / hp2.m_Scale);
    float dist_y = fabs(hp1.m_TranslationY - hp2.m_TranslationY) *
            (0.5f / (hp1.m_Scale * hp1.m_Excentricity) +
             0.5f / (hp2.m_Scale * hp2.m_Excentricity));
    return sqrt(dist_x * dist_x + dist_y * dist_y);
}

bool intersects(const BICV::bicv_HeadPoseParameters& hp1,
        const BICV::bicv_HeadPoseParameters& hp2) {
    return (fabs(hp1.m_TranslationX - hp2.m_TranslationX) <
                ((hp1.m_Scale + hp2.m_Scale) * 100)) &&
           (fabs(hp1.m_TranslationY - hp2.m_TranslationY) <
                ((hp1.m_Scale * hp1.m_Excentricity +
                  hp2.m_Scale * hp2.m_Excentricity) * 100));
}

}

std::ostream& operator<<( std::ostream& os,
        const BICV::bicv_HeadPoseParameters& hp) {
    os << "TX: " << hp.m_TranslationX <<
          ", TY: " << hp.m_TranslationY <<
          ", Scale: " << hp.m_Scale <<
          ", Exc: " << hp.m_Excentricity <<
          ", HP: " << hp.m_HeadPose;
    return os;
}

/// Addition operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise sum of the two head pose parameters structures.
BICV::bicv_HeadPoseParameters operator+(
    const BICV::bicv_HeadPoseParameters& hp1,
    const BICV::bicv_HeadPoseParameters& hp2) {

    BICV::bicv_HeadPoseParameters result = hp1;
    result.m_Excentricity += hp2.m_Excentricity;
    result.m_Scale += hp2.m_Scale;
    result.m_TranslationX += hp2.m_TranslationX;
    result.m_TranslationY += hp2.m_TranslationY;
    result.m_HeadPose += hp2.m_HeadPose;
    return result;
}

/// Subtraction operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise difference (hp1-hp2) of the two head pose parameters structures.
BICV::bicv_HeadPoseParameters operator-(
    const BICV::bicv_HeadPoseParameters& hp1,
    const BICV::bicv_HeadPoseParameters& hp2) {

    BICV::bicv_HeadPoseParameters result = hp1;
    result.m_Excentricity -= hp2.m_Excentricity;
    result.m_Scale -= hp2.m_Scale;
    result.m_TranslationX -= hp2.m_TranslationX;
    result.m_TranslationY -= hp2.m_TranslationY;
    result.m_HeadPose -= hp2.m_HeadPose;
    return result;
}


/// Elementwise multiplication operator.
///
/// @param hp1 Multiplicand head pose parameters structure
/// @param hp2 Multiplier head pose parameters structure
/// @return The elementwise product of the two head pose parameters structures.
BICV::bicv_HeadPoseParameters operator*(
    const BICV::bicv_HeadPoseParameters& hp1,
    const BICV::bicv_HeadPoseParameters& hp2) {
    BICV::bicv_HeadPoseParameters result = hp1;
    result.m_Excentricity *= hp2.m_Excentricity;
    result.m_Scale *= hp2.m_Scale;
    result.m_TranslationX *= hp2.m_TranslationX;
    result.m_TranslationY *= hp2.m_TranslationY;
    result.m_HeadPose *= hp2.m_HeadPose;
    return result;
}

/// Elementwise multiplication operator.
///
/// @param hp Multiplicand head pose parameters structure
/// @param val Multiplier argument
/// @return The elementwise product of the head pose parameters structure.
BICV::bicv_HeadPoseParameters operator*(const BICV::bicv_HeadPoseParameters& hp,
        float val) {
    BICV::bicv_HeadPoseParameters result = hp;
    result.m_Excentricity *= val;
    result.m_Scale *= val;
    result.m_TranslationX *= val;
    result.m_TranslationY *= val;
    result.m_HeadPose *= val;
    return result;
}

/// Elementwise multiplication operator.
///
/// @param hp Multiplicand head pose parameters structure
/// @param val Multiplier argument
/// @return The elementwise product of the head pose parameters structure.
BICV::bicv_HeadPoseParameters operator*(float val,
        const BICV::bicv_HeadPoseParameters& hp) {
    BICV::bicv_HeadPoseParameters result = hp;
    result.m_Excentricity *= val;
    result.m_Scale *= val;
    result.m_TranslationX *= val;
    result.m_TranslationY *= val;
    result.m_HeadPose *= val;
    return result;
}

std::ostream& operator<<( std::ostream& os,
        const BICV::bicv_HeadPoseARTrackerState& hp) {
    os << "Cur(" << hp.m_HeadPoseParamsCur << "), Prev(" <<
          hp.m_HeadPoseParamsPrev << ")";
    return os;
}

/// Addition operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise sum of the two head pose parameters structures.
BICV::bicv_HeadPoseARTrackerState operator+(
    const BICV::bicv_HeadPoseARTrackerState& hp1,
    const BICV::bicv_HeadPoseARTrackerState& hp2) {
    BICV::bicv_HeadPoseARTrackerState result(hp1);
    result.m_HeadPoseParamsCur += hp2.m_HeadPoseParamsCur;
    result.m_HeadPoseParamsPrev += hp2.m_HeadPoseParamsPrev;
    return result;
}

/// Subtraction operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise difference (hp1-hp2) of the two head pose parameters structures.
BICV::bicv_HeadPoseARTrackerState operator-(
    const BICV::bicv_HeadPoseARTrackerState& hp1,
    const BICV::bicv_HeadPoseARTrackerState& hp2) {
    BICV::bicv_HeadPoseARTrackerState result(hp1);
    result.m_HeadPoseParamsCur -= hp2.m_HeadPoseParamsCur;
    result.m_HeadPoseParamsPrev -= hp2.m_HeadPoseParamsPrev;
    return result;
}

/// Multiplication operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise sum of the two head pose parameters structures.
BICV::bicv_HeadPoseARTrackerState operator*(
    const BICV::bicv_HeadPoseARTrackerState& hp1,
    const BICV::bicv_HeadPoseARTrackerState& hp2) {
    BICV::bicv_HeadPoseARTrackerState result(hp1);
    result.m_HeadPoseParamsCur *= hp2.m_HeadPoseParamsCur;
    result.m_HeadPoseParamsPrev *= hp2.m_HeadPoseParamsPrev;
    return result;
}

/// Multiplication operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise sum of the two head pose parameters structures.
BICV::bicv_HeadPoseARTrackerState operator*(
    const BICV::bicv_HeadPoseARTrackerState& hp1,
    float hp2) {
    BICV::bicv_HeadPoseARTrackerState result(hp1);
    result.m_HeadPoseParamsCur *= hp2;
    result.m_HeadPoseParamsPrev *= hp2;
    return result;
}

/// Multiplication operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise sum of the two head pose parameters structures.
BICV::bicv_HeadPoseARTrackerState operator*(float hp2,
    const BICV::bicv_HeadPoseARTrackerState& hp1) {
    BICV::bicv_HeadPoseARTrackerState result(hp1);
    result.m_HeadPoseParamsCur *= hp2;
    result.m_HeadPoseParamsPrev *= hp2;
    return result;
}

/// Multiplication operator.
/// @param hp1 Augend head pose parameters structure
/// @param hp2 Addend head pose parameters structure
/// @return The elementwise sum of the two head pose parameters structures.
BICV::bicv_HeadPoseARTrackerState operator/(
    const BICV::bicv_HeadPoseARTrackerState& hp1,
    float hp2) {
    BICV::bicv_HeadPoseARTrackerState result(hp1);
    result.m_HeadPoseParamsCur /= hp2;
    result.m_HeadPoseParamsPrev /= hp2;
    return result;
}
